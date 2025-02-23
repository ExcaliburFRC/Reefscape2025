// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.State;
import frc.robot.superstructure.Superstructure;
import monologue.Logged;

import static frc.robot.Constants.SwerveConstants.*;

public class RobotContainer implements Logged {
    private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
    // The robot's subsystems and commands are defined here...

    private final Swerve m_swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());
    public final Superstructure m_superstructure = new Superstructure();

    private final CommandPS5Controller m_driver = new CommandPS5Controller(0);
    private final InterpolatingDoubleTreeMap m_decelerator = new InterpolatingDoubleTreeMap();

    private final CommandJoystick m_autoJoystick = new CommandJoystick(1);

    public Runnable updateOdometry = m_swerve::updateOdometry;

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
                                deadband(m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
                                deadband(m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
                        () -> deadband(m_driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC * m_decelerator.get(m_driver.getRawAxis(3)),
                        () -> !m_driver.R2().getAsBoolean()
                )
        );

//        m_driver.povUp().toggleOnTrue(m_swerve.turnToAngleCommand(
//                () -> new Vector2D(
//                        deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
//                        deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
//                Rotation2d::new));
//
//        m_driver.povLeft().toggleOnTrue(m_swerve.turnToAngleCommand(
//                () -> new Vector2D(
//                        deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
//                        deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
//                () -> Rotation2d.fromDegrees(-90)));
//
//        m_driver.povRight().toggleOnTrue(m_swerve.turnToAngleCommand(
//                () -> new Vector2D(
//                        deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
//                        deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
//                ()-> Rotation2d.fromDegrees(90)));
//
//        m_driver.povDown().toggleOnTrue(m_swerve.turnToAngleCommand(
//                () -> new Vector2D(
//                        deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
//                        deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
//                () -> Rotation2d.fromDegrees(180)));

        m_driver.povUp().onTrue(m_superstructure.removeAlgaeCommand(3, () -> true).until(m_driver.R1()).withName("Remove 3"));
        m_driver.povDown().onTrue(m_superstructure.removeAlgaeCommand(2, () -> true).until(m_driver.R1()).withName("Remove 2"));

        m_driver.L1().toggleOnTrue(m_superstructure.intakeCommand(() -> true));

        m_driver.circle().toggleOnTrue(m_superstructure.scoreCoralCommand(2, m_driver.R1()));
        m_driver.square().toggleOnTrue(m_superstructure.scoreCoralCommand(3, m_driver.R1()));

        m_driver.options().onTrue(m_superstructure.ejetCoralCommand());

        m_driver.touchpad().whileTrue(m_superstructure.toggleIdleMode());
        m_driver.create().onTrue(m_superstructure.resetElevator());

        m_driver.PS().onTrue(m_swerve.resetAngleCommand());
    }


    public double deadband(double value) {
        return Math.abs(value) < DEADBAND_VALUE ? 0 : value;
    }

    private void initAutoChooser() {
        EventTrigger releaseCoral = new EventTrigger("atPoseTrigger");
        NamedCommands.registerCommand("scoreL3", m_superstructure.scoreCoralCommand(3, releaseCoral));

//         Build an auto chooser. This will use Commands.none() as the default option.
        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.addOption("Calibration Auto", new PathPlannerAuto("calibrationAuto"));
        m_autoChooser.addOption("1 coral auto", new PathPlannerAuto("1 coral auto"));

        SmartDashboard.putData("Auto Chooser", m_autoChooser);
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
