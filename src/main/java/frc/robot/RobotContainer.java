// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
    // The robot's subsystems and commands are defined here...

    private final Swerve m_swerve = configureSwerve(new Pose2d());

    private final CommandPS5Controller driver = new CommandPS5Controller(0);

    private final InterpolatingDoubleTreeMap m_decelerator = new InterpolatingDoubleTreeMap();

    public Runnable updateOdometry = m_swerve::updateOdometry;

    private SendableChooser<Command> m_autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_decelerator.put(-1.0, 1.0);
        m_decelerator.put(1.0, 0.25);

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
                        () -> true
                ));

        driver.PS().onTrue(resetAngleCommand());

        driver.cross().onTrue(
                m_swerve.driveToPoseWithOverrideCommand(
                        new Pose2d(1, 0, new Rotation2d()),
                        () -> new Vector2D(
                                deadband(-driver.getLeftY()) * MAX_VEL * m_decelerator.get(driver.getRawAxis(3)),
                                deadband(-driver.getLeftX()) * MAX_VEL * m_decelerator.get(driver.getRawAxis(3))),
                        () -> deadband(-driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC
                )
        );
    }


    public double deadband(double value) {
        return Math.abs(value) < 0.15 ? 0 : value;
    }

    private void initAutoChooser() {
        NamedCommands.registerCommand("print command", new PrintCommand("pathplanner"));

        new EventTrigger("testTrigger").whileTrue(Commands.run(() -> System.out.println("Trigger Test")));
        new PointTowardsZoneTrigger("PointTowardsZoneTrigger").whileTrue(Commands.run(() -> System.out.println("Trigger Test2")));

        // Build an auto chooser. This will use Commands.none() as the default option.
        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.addOption("Test Auto", new PathPlannerAuto("testAuto"));
        m_autoChooser.addOption("Test Auto 2", new PathPlannerAuto("testAuto2"));
        m_autoChooser.addOption("Calibration Auto", new PathPlannerAuto("calibrationAuto"));
        m_autoChooser.addOption("Test Choreo Auto", new PathPlannerAuto("testChoreoAuto"));
        m_autoChooser.addOption("Test Trigger", new PathPlannerAuto("triggerTest"));
        m_autoChooser.addOption("Heart", new PathPlannerAuto("HeartAuto"));

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    private void initElastic() {
        PowerDistribution PDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
        SmartDashboard.putData("PDH", PDH);

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
        SendableChooser<Rotation2d> angleChooser = new SendableChooser<>();
        angleChooser.setDefaultOption("0", new Rotation2d());
        angleChooser.addOption("90", new Rotation2d(Math.PI / 2));
        angleChooser.addOption("180", new Rotation2d(Math.PI));
        angleChooser.addOption("270", new Rotation2d(3 * Math.PI / 2));
        swerveTab.add("Angle Chooser", angleChooser);
        swerveTab.add("Turn To Angle",
                m_swerve.turnToAngleCommand(
                        () -> new Vector2D(
                                deadband(-driver.getLeftY()) * MAX_VEL * m_decelerator.get(driver.getRawAxis(3)),
                                deadband(-driver.getLeftX()) * MAX_VEL * m_decelerator.get(driver.getRawAxis(3)
                                )
                        ),
                        angleChooser::getSelected
                ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
