package frc.excalib.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.excalib.additional_utilities.Elastic;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.math.Vector2D;
import frc.excalib.slam.mapper.Odometry;
import monologue.Logged;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.kTextView;
import static edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelSelf;
import static frc.excalib.additional_utilities.Elastic.Notification.NotificationLevel.WARNING;
import static frc.excalib.swerve.SwerveAccUtils.getSmartTranslationalVelocitySetPoint;
import static frc.robot.Constants.SwerveConstants.*;
import static monologue.Annotations.*;

/**
 * A class representing a swerve subsystem.
 */
public class Swerve extends SubsystemBase implements Logged {
    private final ModulesHolder m_MODULES;
    private final IMU m_imu;
    private final Odometry m_odometry;
    private final SwerveDriveKinematics m_swerveDriveKinematics;

    public final Field2d m_field = new Field2d();

    /**
     * A constructor that initialize the Swerve Subsystem
     *
     * @param modules         The ModulesHolder containing all swerve modules.
     * @param imu             IMU sensor.
     * @param initialPosition The initial position of the robot.
     */
    public Swerve(ModulesHolder modules,
                  IMU imu,
                  Pose2d initialPosition) {

        this.m_MODULES = modules;
        this.m_imu = imu;
        m_imu.resetIMU();

        // Initialize odometry with the current yaw angle
        this.m_odometry = new Odometry(
                modules.getSwerveDriveKinematics(),
                modules.getModulesPositions(),
                this::getRotation2D,
                initialPosition
        );

        m_swerveDriveKinematics = new SwerveDriveKinematics(
                FRONT_LEFT_TRANSLATION,
                FRONT_RIGHT_TRANSLATION,
                BACK_LEFT_TRANSLATION,
                BACK_RIGHT_TRANSLATION
        );

        initAutoBuilder();
    }

    /**
     * Creates a drive command for the swerve drive.
     *
     * @param velocityMPS    Supplier for the desired linear velocity in meters per second.
     * @param omegaRadPerSec Supplier for the desired rotational velocity in radians per second.
     * @param fieldOriented  Supplier indicating whether the control is field-oriented.
     * @return A command that drives the robot.
     */
    public Command driveCommand(
            Supplier<Vector2D> velocityMPS,
            DoubleSupplier omegaRadPerSec,
            BooleanSupplier fieldOriented) {

        // Precompute values to avoid redundant calculations
        Supplier<Vector2D> adjustedVelocitySupplier = () -> {
//            Vector2D velocity = velocityMPS.get();
            Vector2D velocity = getSmartTranslationalVelocitySetPoint(getVelocity(), velocityMPS.get());
            if (fieldOriented.getAsBoolean()) {
                Rotation2d yaw = getRotation2D().unaryMinus();
                return velocity.rotate(yaw);
            }
            return velocity;
        };

        Command driveCommand = m_MODULES.setVelocitiesCommand(
                adjustedVelocitySupplier,
                omegaRadPerSec
        );
        driveCommand.setName("Drive Command");
        driveCommand.addRequirements(this);
        return driveCommand;
    }

    /**
     * A non-command function that drives the robot (for pathplanner).
     *
     * @param speeds A ChassisSpeeds object represents ROBOT RELATIVE speeds desired speeds.
     */
    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        m_MODULES.setModulesStates(m_swerveDriveKinematics.toSwerveModuleStates(speeds));
    }

    /**
     * A method that turns the robot to a desired angle.
     *
     * @param angle The desired angle in radians.
     * @return A command that turns the robot to the wanted angle.
     */
    public Command turnToAngleCommand(Supplier<Vector2D> velocityMPS, Supplier<Rotation2d> angle) {
        PIDController angleController = new PIDController(ANGLE_PID_CONSTANTS.kP, ANGLE_PID_CONSTANTS.kI, ANGLE_PID_CONSTANTS.kD);
        angleController.enableContinuousInput(0, 2 * Math.PI);
        angleController.setTolerance(0.07);
        return driveCommand(
                velocityMPS,
                () -> angleController.calculate(getPose2D().getRotation().getRadians(), angle.get().getRadians()),
                () -> true
        ).until(angleController::atSetpoint);
    }

    public Command driveToPoseCommand(Pose2d setPoint) {
        return AutoBuilder.pathfindToPose(
                setPoint,
                new PathConstraints(MAX_VEL, MAX_FORWARD_ACC, MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC, 12.0, false)
        );
    }

    public Command pathfindThenFollowPathCommand(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            Elastic.sendNotification(new Elastic.Notification(
                    WARNING,
                    "Path Creating Error",
                    "the path file " + pathName + " doesn't exist")
            );
            return new PrintCommand("this path file doesn't exist");
        }

        return AutoBuilder.pathfindThenFollowPath(
                path,
                new PathConstraints(MAX_VEL, MAX_FORWARD_ACC, MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC, 12.0, false)
        );
    }

    /**
     * Updates the robot's odometry.
     */
    public void updateOdometry() {
        m_odometry.updateOdometry(m_MODULES.getModulesPositions());
    }

    /**
     * A method that restarts the odometry.
     *
     * @param newPose the wanted new Pose2d of the robot.
     */
    public void resetOdometry(Pose2d newPose) {
        m_odometry.resetOdometry(m_MODULES.getModulesPositions(), newPose);
    }

    /**
     * Gets the robot's rotation.
     *
     * @return The current rotation of the robot.
     */
    @Log.NT
    public Rotation2d getRotation2D() {
        return m_imu.getZRotation();
    }

    /**
     * Gets the robot's pose.
     *
     * @return The current pose of the robot.
     */
    @Log.NT
    public Pose2d getPose2D() {
        return m_odometry.getRobotPose();
    }

    /**
     * Gets the current velocity of the robot.
     *
     * @return The robot's velocity as a Vector2D.
     */
    public Vector2D getVelocity() {
        return m_MODULES.getVelocity();
    }

    /**
     * Gets the current robot relative speed.
     *
     * @return The robot's speed as a ChassisSpeeds.
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_swerveDriveKinematics.toChassisSpeeds(m_MODULES.logStates());
    }

    /**
     * A function that initialize the AutoBuilder for pathplanner.
     */
    private void initAutoBuilder() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose2D, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelativeChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 1.0), // Translation PID constants
                        ANGLE_PID_CONSTANTS // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void initElastic() {
        SmartDashboard.putData("Swerve Drive", builder -> {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> m_MODULES.m_frontLeft.getPosition().getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> m_MODULES.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Front Right Angle", () -> m_MODULES.m_frontRight.getPosition().getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> m_MODULES.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Back Left Angle", () -> m_MODULES.m_backLeft.getPosition().getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> m_MODULES.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Back Right Angle", () -> m_MODULES.m_backRight.getPosition().getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> m_MODULES.m_frontLeft.getVelocity().getDistance(), null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation2D().getRadians(), null);
        });

        SmartDashboard.putData("Field", m_field);

        SmartDashboard.putData("swerve info", this);

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

        GenericEntry odometryXEntry = swerveTab.add("odometryX", 0).withWidget(kTextView).getEntry();
        GenericEntry odometryYEntry = swerveTab.add("odometryY", 0).withWidget(kTextView).getEntry();
        GenericEntry odometryAngleEntry = swerveTab.add("odometryAngle", 0).withWidget(kTextView).getEntry();

        swerveTab.add("Reset Odometry",
                new InstantCommand(
                        () -> resetOdometry(
                                new Pose2d(
                                        odometryXEntry.getDouble(0),
                                        odometryYEntry.getDouble(0),
                                        Rotation2d.fromDegrees(odometryAngleEntry.getDouble(0)))
                        ), this).ignoringDisable(true)
        );

        GenericEntry OTFGxEntry = swerveTab.add("OTFGx", 0).withWidget(kTextView).getEntry();
        GenericEntry OTFGyYEntry = swerveTab.add("OTFGy", 0).withWidget(kTextView).getEntry();
        GenericEntry OTFGAngleEntry = swerveTab.add("OTFGAngle", 0).withWidget(kTextView).getEntry();
        swerveTab.add("Drive To Pose", driveToPoseCommand(
                        new Pose2d(
                                OTFGxEntry.getDouble(0),
                                OTFGyYEntry.getDouble(0),
                                Rotation2d.fromDegrees(OTFGAngleEntry.getDouble(0)))
                )
        );
    }

    /**
     * Runs a system identification routine on a specific module's angle.
     *
     * @param module  The module index (0-3).
     * @param dir     The direction of the sysid routine.
     * @param dynamic Whether to perform a dynamic or quasistatic test.
     * @return The command to perform the sysid routine.
     */
    public Command driveSysId(int module, Direction dir, boolean dynamic) {
        SwerveModule selectedModule;

        switch (module) {
            case 0 -> selectedModule = m_MODULES.m_frontLeft;
            case 1 -> selectedModule = m_MODULES.m_frontRight;
            case 2 -> selectedModule = m_MODULES.m_backLeft;
            case 3 -> selectedModule = m_MODULES.m_backRight;
            default -> {
                throw new IllegalArgumentException("Invalid module index: " + module);
            }
        }

        return dynamic ?
                selectedModule.angleSysIdDynamic(dir, this)
                : selectedModule.angleSysIdQuas(dir, this);
    }

    @Override
    public void periodic() {
        updateOdometry();
        m_field.setRobotPose(getPose2D());
    }
}
