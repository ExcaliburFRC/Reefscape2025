// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.gripper.Gripper;
import monologue.Logged;

import static frc.robot.Constants.SwerveConstants.*;

public class RobotContainer implements Logged {
    private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
    // The robot's subsystems and commands are defined here...

    private final Swerve m_swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());

    private final CommandPS5Controller m_driver = new CommandPS5Controller(0);

    private final Gripper m_gripper = new Gripper();

    private final InterpolatingDoubleTreeMap m_decelerator = new InterpolatingDoubleTreeMap();

//    public Runnable updateOdometry = m_swerve::updateOdometry;

    private SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        m_decelerator.put(-1.0, 1.0);
        m_decelerator.put(1.0, 0.25);

        initAutoChooser();
        initElastic();

        // Configure the trigger bindings
        configureBindings();
    }


    private void configureBindings() {


        m_swerve.setDefaultCommand(
                m_swerve.driveCommand(
                        () -> new Vector2D(
                                deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
                                deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
                        () -> deadband(-m_driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                ));

        m_driver.PS().onTrue(m_swerve.resetAngleCommand());
        /*
        m_driver.cross().onTrue(
                m_swerve.driveToPoseWithOverrideCommand(
                        new Pose2d(1, 0, new Rotation2d()),
                        m_driver.R2(),
                        () -> new Vector2D(
                                deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
                                deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
                        () -> deadband(-m_driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC
                )
        );*/

        m_driver.circle().toggleOnTrue(m_swerve.driveSysId(1, SysIdRoutine.Direction.kForward, false));
        m_driver.cross().toggleOnTrue(m_swerve.driveSysId(1, SysIdRoutine.Direction.kReverse, false));
        m_driver.triangle().toggleOnTrue(m_swerve.driveSysId(1, SysIdRoutine.Direction.kForward, true));
        m_driver.square().toggleOnTrue(m_swerve.driveSysId(1, SysIdRoutine.Direction.kReverse, true));

    }

    public double deadband(double value) {
        return Math.abs(value) < DEADBAND_VALUE ? 0 : value;
    }

    private void initAutoChooser() {
        NamedCommands.registerCommand("print command", new PrintCommand("pathplanner"));

        new EventTrigger("testTrigger").whileTrue(Commands.run(() -> System.out.println("Trigger Test")));
        new PointTowardsZoneTrigger("PointTowardsZoneTrigger").whileTrue(Commands.run(() -> System.out.println("Trigger Test2")));

        // Build an auto chooser. This will use Commands.none() as the default option.
//        m_autoChooser = AutoBuilder.buildAutoChooser();
//        m_autoChooser.addOption("Test Auto", new PathPlannerAuto("testAuto"));
//        m_autoChooser.addOption("Test Auto 2", new PathPlannerAuto("testAuto2"));
//        m_autoChooser.addOption("Calibration Auto", new PathPlannerAuto("calibrationAuto"));
//        m_autoChooser.addOption("Test Choreo Auto", new PathPlannerAuto("testChoreoAuto"));
//        m_autoChooser.addOption("Test Trigger", new PathPlannerAuto("triggerTest"));
//        m_autoChooser.addOption("Heart", new PathPlannerAuto("HeartAuto"));
//
//        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    private void initElastic() {
        PowerDistribution PDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
        SmartDashboard.putData("PDH", PDH);

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
        SendableChooser<Rotation2d> angleChooser = new SendableChooser<>();

        // angle chooser options
        angleChooser.setDefaultOption("0", new Rotation2d());
        angleChooser.addOption("90", new Rotation2d(Math.PI / 2));
        angleChooser.addOption("180", new Rotation2d(Math.PI));
        angleChooser.addOption("270", new Rotation2d(3 * Math.PI / 2));

        swerveTab.add("Angle Chooser", angleChooser);

//        swerveTab.add("Turn To Angle",
//                m_swerve.turnToAngleCommand(
//                        () -> new Vector2D(
//                                deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
//                                deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)
//                                )
//                        ),
//                        angleChooser::getSelected
//                ));
    }


    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
