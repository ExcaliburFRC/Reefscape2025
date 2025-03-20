package frc.excalib.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.Elastic;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.math.Vector2D;
import frc.excalib.slam.mapper.Odometry;
import frc.excalib.slam.mapper.PhotonAprilTagsCamera;
import frc.robot.Constants;
import monologue.Logged;
import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.apriltag.AprilTagFields.k2025Reefscape;
import static edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets.kTextView;
import static frc.excalib.additional_utilities.Elastic.Notification.NotificationLevel.WARNING;
import static frc.robot.Constants.FieldConstants.BLUE_REEF_CENTER;
import static frc.robot.Constants.FieldConstants.RED_REEF_CENTER;
import static frc.robot.Constants.SwerveConstants.*;
import static monologue.Annotations.Log;

/**
 * A class representing a swerve subsystem.
 */
public class Swerve extends SubsystemBase implements Logged {
    private final ModulesHolder m_MODULES;
    private final IMU m_imu;
    private final Odometry m_odometry;
    private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();
    private final PhotonAprilTagsCamera m_frontCamera, m_backCamera;
    private Trigger finishTrigger;
    private Rotation2d pi = new Rotation2d(Math.PI);
    private InterpolatingDoubleTreeMap velocityLimit = new InterpolatingDoubleTreeMap();

//    private AuroraClient m_auroraClient = new AuroraClient(5800);

    private final SwerveDriveKinematics m_swerveDriveKinematics;

    private final PIDController m_angleController = new PIDController(ANGLE_PID_GAINS.kp, ANGLE_PID_GAINS.ki, ANGLE_PID_GAINS.kd);
    private final PIDController m_xController = new PIDController(
            TRANSLATION_PID_GAINS.kp, TRANSLATION_PID_GAINS.ki, TRANSLATION_PID_GAINS.kd
//            new TrapezoidProfile.Constraints(MAX_VEL, 6)
    );
    private final PIDController m_yController = new PIDController(
            TRANSLATION_PID_GAINS.kp, TRANSLATION_PID_GAINS.ki, TRANSLATION_PID_GAINS.kd
//            new TrapezoidProfile.Constraints(MAX_VEL, 6)
    );
    public final Field2d m_field = new Field2d();
    private Supplier<Rotation2d> m_angleSetpoint = Rotation2d::new;
    private Supplier<Translation2d> m_translationSetpoint = Translation2d::new;

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

        m_angleController.enableContinuousInput(-Math.PI, Math.PI);
        m_angleController.setTolerance(0.026);
        m_xController.setTolerance(0.01);
        m_yController.setTolerance(0.01);

        finishTrigger = new Trigger(m_xController::atSetpoint).and(m_yController::atSetpoint).and(m_angleController::atSetpoint).debounce(1);
        // Initialize odometry with the current yaw angle
        this.m_odometry = new Odometry(
                modules.getSwerveDriveKinematics(),
                modules.getModulesPositions(),
                m_imu::getZRotation,
                initialPosition
        );
        m_frontCamera = new PhotonAprilTagsCamera("Front", k2025Reefscape, new Transform3d(0.08671062685, 0.28129984689, 0.359, new Rotation3d(0, 0, Math.toRadians(-30.96227128))));
        m_backCamera = new PhotonAprilTagsCamera("Back", k2025Reefscape, new Transform3d(-0.1455338456, 0.2979238, 0.94232478177, new Rotation3d(0, Math.toRadians(-48), Math.toRadians(180))));

        m_swerveDriveKinematics = m_MODULES.getSwerveDriveKinematics();
        velocityLimit.put(0.1, 0.3);
        velocityLimit.put(0.4, 0.6);
        velocityLimit.put(1.5, MAX_VEL);

        initAutoBuilder();
        initElastic();
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
            Vector2D velocity = velocityMPS.get();
//            Vector2D velocity = getSmartTranslationalVelocitySetPoint(getVelocity(), velocityMPS.get());
            if (fieldOriented.getAsBoolean()) {
                Rotation2d yaw = getRotation2D().unaryMinus();
                if (!AllianceUtils.isBlueAlliance()) yaw = yaw.plus(pi);
                return velocity.rotate(yaw);
            }
            return velocity;
        };

        Command driveCommand = new ParallelCommandGroup(m_MODULES.setVelocitiesCommand(
                adjustedVelocitySupplier,
                omegaRadPerSec
        ),
                new RunCommand(
                        () -> {
                            m_desiredChassisSpeeds = new ChassisSpeeds(
                                    adjustedVelocitySupplier.get().getX(),
                                    adjustedVelocitySupplier.get().getY(),
                                    omegaRadPerSec.getAsDouble());
                        }
                )
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
     * @param angleSetpoint The desired angle in radians.
     * @return A command that turns the robot to the wanted angle.
     */
    public Command turnToAngleCommand(Supplier<Vector2D> velocityMPS, Supplier<Rotation2d> angleSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_angleSetpoint = angleSetpoint),
                driveCommand(
                        velocityMPS,
                        () -> m_angleController.calculate(getRotation2D().getRadians(), angleSetpoint.get().getRadians()),
                        () -> true
                )
        ).withName("Turn To Angle");
    }

    public Command pidToPoseCommand(Supplier<Pose2d> poseSetpoint) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            m_xController.calculate(getPose2D().getX(), poseSetpoint.get().getX());
                            m_yController.calculate(getPose2D().getY(), poseSetpoint.get().getY());
                            m_angleController.calculate(getRotation2D().getRadians(), poseSetpoint.get().getRotation().getRadians());
                            m_translationSetpoint = () -> poseSetpoint.get().getTranslation();
                            m_angleSetpoint = () -> poseSetpoint.get().getRotation();
                        }
                ),
                driveCommand(
                        () -> {
                            Vector2D vel = new Vector2D(
                                    m_xController.calculate(getPose2D().getX(), poseSetpoint.get().getX()),
                                    m_yController.calculate(getPose2D().getY(), poseSetpoint.get().getY())
                            );
                            double distance = getPose2D().getTranslation().getDistance(poseSetpoint.get().getTranslation());
                            vel.setMagnitude(Math.min(vel.getDistance(), velocityLimit.get(distance)));
//                            vel = vel.rotate(poseSetpoint.get().getRotation());
//                            vel.setX(Math.signum(vel.getX()) * Math.min(Math.abs(vel.getX()), 1.2));
//                            vel.setY(Math.signum(vel.getY()) * Math.min(Math.abs(vel.getY()), 0.4));
//                            vel = vel.rotate(poseSetpoint.get().getRotation().unaryMinus());
                            if (!AllianceUtils.isBlueAlliance()) return vel.rotate(pi);
                            return vel;
                        },
                        () -> m_angleController.calculate(getRotation2D().getRadians(), poseSetpoint.get().getRotation().getRadians()),
                        () -> true
                )
        ).until(finishTrigger).withName("PID To Pose");
    }

    /**
     * A method that drives the robot to a desired pose.
     *
     * @param setPoint The desired pose.
     * @return A command that drives the robot to the wanted pose.
     */
    public Command driveToPoseCommand(Pose2d setPoint) {
        return AutoBuilder.pathfindToPose(
                setPoint,
                MAX_PATH_CONSTRAINTS
        ).withName("Pathfinding Command");
    }

    public Command driveToPoseWithOverrideCommand(
            Pose2d setPoint,
            BooleanSupplier override,
            Supplier<Vector2D> velocityMPS,
            DoubleSupplier omegaRadPerSec) {
        Command driveToPoseCommand = driveToPoseCommand(setPoint);
        return new SequentialCommandGroup(
                driveToPoseCommand.until(() -> velocityMPS.get().getDistance() != 0 && override.getAsBoolean()),
                driveCommand(
                        velocityMPS,
                        omegaRadPerSec,
                        () -> true
                ).until(() -> velocityMPS.get().getDistance() == 0)
        ).repeatedly().until(driveToPoseCommand::isFinished).withName("Pathfinding With Override Command");
    }

    /**
     * A method that drives the robot to the starting pose of a path, then follows the path.
     *
     * @param pathName The path which the robot needs to follow.
     * @return A command that turns the robot to the wanted angle.
     */
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
                MAX_PATH_CONSTRAINTS
        );
    }

    public Command resetAngleCommand() {
        return new InstantCommand(m_imu::resetIMU).ignoringDisable(true);
    }

    public Command coastCommand() {
        Command coastCommand = m_MODULES.coastCommand().ignoringDisable(true).withName("Coast Command");
        coastCommand.addRequirements(this);
        return coastCommand;
    }

    /**
     * Updates the robot's odometry.
     */
    public void updateOdometry() {
        m_odometry.updateOdometry(m_MODULES.getModulesPositions());

        Optional<EstimatedRobotPose> backPose = m_backCamera.getEstimatedGlobalPose(m_odometry.getEstimatedPosition());
        if (backPose.isPresent()) {
            m_odometry.addVisionMeasurement(backPose.get().estimatedPose.toPose2d(), backPose.get().timestampSeconds);
        }

        Optional<EstimatedRobotPose> frontPose = m_frontCamera.getEstimatedGlobalPose(m_odometry.getEstimatedPosition());
        if (frontPose.isPresent()) {
            m_odometry.addVisionMeasurement(frontPose.get().estimatedPose.toPose2d(), frontPose.get().timestampSeconds);
        }
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
    @Log.NT(key = "Robot Rotation")
    public Rotation2d getRotation2D() {
        return getPose2D().getRotation();
    }

    @Log.NT(key = "Angle Setpoint")
    public Rotation2d getAngleSetpoint() {
        return m_angleSetpoint.get();
    }

    @Log.NT(key = "Translation Setpoint")
    public Translation2d getTranslationSetpoint() {
        return m_translationSetpoint.get();
    }

    /**
     * Gets the robot's pose.
     *
     * @return The current pose of the robot.
     */
    @Log.NT(key = "Robot Pose")
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
     * Gets the current acceleration of the robot.
     *
     * @return The robot's acceleration as a Vector2D.
     */
    @Log.NT(key = "Acceleration")
    public double getAccelerationDistance() {
        return new Vector2D(m_imu.getAccX(), m_imu.getAccY()).getDistance();
    }

    /**
     * Gets the current robot relative speed.
     *
     * @return The robot's speed as a ChassisSpeeds.
     */
    @Log.NT(key = "Measured Chassis Speeds")
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_swerveDriveKinematics.toChassisSpeeds(m_MODULES.logStates());
    }

    @Log.NT
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return m_desiredChassisSpeeds;
    }

    @Log.NT
    public double distanceFromReefCenter() {
        return AllianceUtils.isBlueAlliance() ?
                BLUE_REEF_CENTER.getDistance(getPose2D().getTranslation()) :
                RED_REEF_CENTER.getDistance(getPose2D().getTranslation());
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
                        TRANSLATION_PID_PP_CONSTANTS, // Translation PID constants
                        ANGLE_PID_PP_CONSTANTS // Rotation PID constants
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

    /**
     * A function that initialize the Swerve tab for Elastic.
     */
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
        GenericEntry OTFGyEntry = swerveTab.add("OTFGy", 0).withWidget(kTextView).getEntry();
        GenericEntry OTFGAngleEntry = swerveTab.add("OTFGAngle", 0).withWidget(kTextView).getEntry();
        swerveTab.add("Drive To Pose",
                driveToPoseCommand(
                        new Pose2d(
                                OTFGxEntry.getDouble(0),
                                OTFGyEntry.getDouble(0),
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
    public Command driveSysId(int module, Direction dir, SysidConfig sysidConfig, boolean dynamic) {
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
                selectedModule.driveSysIdDynamic(dir, this, sysidConfig)
                : selectedModule.driveSysIdQuas(dir, this, sysidConfig);
    }

    /**
     * Runs a system identification routine on a specific module's angle.
     *
     * @param module  The module index (0-3).
     * @param dir     The direction of the sysid routine.
     * @param dynamic Whether to perform a dynamic or quasistatic test.
     * @return The command to perform the sysid routine.
     */
    public Command angleSysId(int module, Direction dir, SysidConfig sysidConfig, boolean dynamic) {
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
                selectedModule.angleSysIdDynamic(dir, this, sysidConfig)
                : selectedModule.angleSysIdQuas(dir, this, sysidConfig);
    }

    @Override
    public void periodic() {
        m_MODULES.periodic();
//        updateOdometry();
        m_field.setRobotPose(getPose2D());
    }
}
