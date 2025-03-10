// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.additional_utilities.Color;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.Superstructure;
import monologue.Logged;

import static frc.robot.Constants.NET_ID;
import static frc.robot.Constants.PROCESSOR_ID;
import static frc.robot.Constants.SwerveConstants.*;
import static monologue.Annotations.Log.NT;

public class RobotContainer implements Logged {
    private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
    // The robot's subsystems and commands are defined here...

    private final Swerve m_swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());
    private Superstructure m_superstructure = new Superstructure();
//    private final LEDs leds = LEDs.getInstance();


    private final CommandPS5Controller m_driver = new CommandPS5Controller(0);
    private final CommandPS5Controller m_test = new CommandPS5Controller(1);
    private final InterpolatingDoubleTreeMap m_decelerator = new InterpolatingDoubleTreeMap();
    private final Automations automations;
//    public Runnable updateOdometry = m_swerve::updateOdometry;

    private SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        m_decelerator.put(-1.0, 1.0);
        m_decelerator.put(1.0, 0.25);
        automations = new Automations(m_swerve, m_superstructure);


//        initAutoChooser();
        initElastic();

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        /*m_driver.triangle().onTrue(automations.L4Command(true));
        m_driver.povUp().onTrue(automations.L4Command(false));

        m_driver.square().onTrue(automations.L3Command(true));
        m_driver.povRight().onTrue(automations.L3Command(false));

        m_driver.circle().onTrue(automations.L2Command(true));
        m_driver.povLeft().onTrue(automations.L2Command(false));

        m_driver.cross().onTrue(automations.L1Command(true));
        m_driver.povDown().onTrue(automations.L1Command(false));

        m_driver.L1().onTrue(m_superstructure.intakeCoralCommand());
        m_driver.R1().onTrue(m_superstructure.collapseCommand());

        m_driver.L2().onTrue(automations.intakeAlgaeCommand());
*/

//        m_driver.circle().onTrue(m_superstructure.intakeCoralCommand().raceWith(leds.setPattern(LEDs.LEDPattern.BLINKING, Color.Colors.GREEN.color)));
//        m_driver.cross().onTrue(automations.scoreCoralCommand(4, false));

//        m_driver.square().onTrue(automations.L3Command(true));
//        m_driver.triangle().onTrue(automations.L4Command(true));
        m_driver.cross().onTrue(automations.L1Command(true));
        m_driver.povRight().onTrue(automations.L3Command(false));
        m_driver.povUp().onTrue(automations.L4Command(false));
        m_driver.povDown().onTrue(automations.L1Command(false));

        m_driver.R2().onTrue(
//                m_superstructure.hasAlgaeTrigger().getAsBoolean() ?
//                        automations.scoreAlgaeCommand() :
                        automations.intakeAlgaeCommand()
        );

        m_driver.L1().onTrue(m_superstructure.intakeCoralCommand()
                //.andThen(m_superstructure.collapseCommand())
        );
//        m_driver.R1().onTrue(m_superstructure.collapseCommand());
        m_driver.create().onTrue(m_superstructure.collapseCommand());
        //options - force outake coral, r1 - outake coral

//        m_driver.touchpad().whileTrue(m_superstructure.coastCommand());
//        m_driver.circle().onTrue(m_swerve.pidToPoseCommand(() -> new Pose2d(1.5, 1, new Rotation2d(Math.PI /3))));
        m_swerve.setDefaultCommand(
                m_swerve.driveCommand(
                        () -> new Vector2D(
                                deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
                                deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
                        () -> deadband(-m_driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC * (!m_driver.L2().getAsBoolean() ? m_decelerator.get(m_driver.getRawAxis(3)) : 0.065),
                        () -> true
                )
        );

        m_driver.PS().onTrue(m_swerve.resetAngleCommand());

        m_driver.touchpad().whileTrue(m_superstructure.coastCommand());
    }


    public double deadband(double value) {
        return Math.abs(value) < DEADBAND_VALUE ? 0 : value;
    }

    private void initAutoChooser() {
        EventTrigger releaseCoral = new EventTrigger("atPoseTrigger");

//         Build an auto chooser. This will use Commands.none() as the default option.
        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.addOption("Calibration Auto", new PathPlannerAuto("calibrationAuto"));
        m_autoChooser.addOption("Test Path", new PathPlannerAuto("testAuto"));
        m_autoChooser.addOption("Test Auto", new PathPlannerAuto("testAuto2"));

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

        SmartDashboard.putData("toggleHasCoral", m_superstructure.toggleCoralCommand());

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
        return Commands.none(); //m_autoChooser.getSelected();
    }

    @NT
    public double getLeftY() {
        return m_driver.getLeftY();
    }

    @NT
    public double getLeftX() {
        return m_driver.getLeftX();
    }

    @NT
    public double getRightX() {
        return m_driver.getRightX();
    }
}
