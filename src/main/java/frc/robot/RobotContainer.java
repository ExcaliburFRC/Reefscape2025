// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import monologue.Logged;

import static frc.robot.Constants.SwerveConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {
    // The robot's subsystems and commands are defined here...

    private final Swerve m_swerve = configureSwerve(new Pose2d());

    private final CommandPS5Controller driver = new CommandPS5Controller(0);

    private final InterpolatingDoubleTreeMap m_decelerator = new InterpolatingDoubleTreeMap();

    public Runnable updateOdometry = m_swerve::updateOdometry;

    private SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_decelerator.put(-1.0, 1.0);
        m_decelerator.put(1.0, 0.25);

        m_swerve.initShuffleboard();

        initAutoChooser();
        initElastic();

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        m_swerve.setDefaultCommand(
                m_swerve.driveCommand(
                        () -> new Vector2D(
                                deadband(-driver.getLeftY()) * MAX_VEL * m_decelerator.get(driver.getRawAxis(3)),
                                deadband(-driver.getLeftX()) * MAX_VEL * m_decelerator.get(driver.getRawAxis(3))),
                        () -> deadband(-driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true)
        );

        driver.PS().onTrue(resetSwerveCommand());

//        GenericEntry angleEntry = Shuffleboard.getTab("Swerve").add("Angle", 0).getEntry();
//        GenericEntry poseEntry = Shuffleboard.getTab("Swerve").add("Pose", new Pose2d()).getEntry();

        driver.triangle().onTrue(m_swerve.turnToAngleCommand(() -> new Rotation2d(Math.PI)/*Rotation2d.fromDegrees(angleEntry.getDouble(0))*/));

        driver.cross().onTrue(m_swerve.driveToPoseCommand(new Pose2d(1, 1, new Rotation2d(Math.PI / 2))));
    }

    public double deadband(double value) {
        return Math.abs(value) < 0.1 ? 0 : value;
    }

    private void initAutoChooser() {
        NamedCommands.registerCommand("print command", new PrintCommand("pathplanner"));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Test Auto", new PathPlannerAuto("testAuto"));
        autoChooser.addOption("Test Auto 2", new PathPlannerAuto("testAuto2"));
        autoChooser.addOption("Calibration Auto", new PathPlannerAuto("calibrationAuto"));
        autoChooser.addOption("Test Choreo Auto", new PathPlannerAuto("testChoreoAuto"));

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void initElastic() {
        PowerDistribution PDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
        SmartDashboard.putData("PDH", PDH);

        SmartDashboard.putData("Field", m_swerve.m_field);

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        Shuffleboard.getTab("Swerve").add("Reset Odometry", new InstantCommand(() -> m_swerve.restartOdometry(new Pose2d()), m_swerve).ignoringDisable(true));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
