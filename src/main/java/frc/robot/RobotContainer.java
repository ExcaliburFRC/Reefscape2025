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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.control.math.MathUtils;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.automations.Automations;
import frc.robot.automations.Slice;
import frc.robot.superstructure.Superstructure;
import monologue.Annotations;
import monologue.Annotations.Log;
import monologue.Logged;

import static frc.robot.Constants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.automations.Constants.FieldConstants.getReefCenter;
import static monologue.Annotations.*;

public class RobotContainer implements Logged {
    private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
    // The robot's subsystems and commands are defined here...

    private final Swerve m_swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());
    private Superstructure m_superstructure = new Superstructure();
    private final LEDs leds = LEDs.getInstance();
    private boolean right;

    public final Runnable m_odometryUpdater = m_swerve::updateOdometry;


    private final CommandPS5Controller m_driver = new CommandPS5Controller(0);
    private final CommandPS5Controller m_operator = new CommandPS5Controller(1);
    private final InterpolatingDoubleTreeMap m_decelerator = new InterpolatingDoubleTreeMap();
    private final Automations m_automations;


    public SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public RobotContainer() {
        m_decelerator.put(-1.0, 1.0);
        m_decelerator.put(1.0, 0.25);
        m_automations = new Automations(m_swerve, m_superstructure);


        initAutoChooser();
        initElastic();

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {

        m_swerve.setDefaultCommand(
                new SequentialCommandGroup(
                        m_automations.autoSwerveCommand().until(m_automations.m_autoMode.negate()),
                        m_swerve.driveCommand(
                                () -> new Vector2D(
                                        deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
                                        deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
                                () -> deadband(-m_driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC * (!m_driver.L2().getAsBoolean() ? m_decelerator.get(m_driver.getRawAxis(3)) : 0.065),
                                () -> true
                        ).until(m_automations.m_autoMode)
                )
        );

        m_driver.povDown().toggleOnTrue(m_automations.L1Command());

        m_driver.square().toggleOnTrue(
                new ConditionalCommand(
                        m_automations.L3Command(true),
                        m_automations.L3Command(false),
                        () -> right
                )
        );

        m_driver.triangle().toggleOnTrue(
                new ConditionalCommand(
                        m_automations.L4Command(true),
                        m_automations.L4Command(false),
                        () -> right
                )
        );

        m_driver.cross().toggleOnTrue(m_automations.L1Command());

        m_driver.povRight().onTrue(m_automations.toggleAutoMode());
        m_driver.povUp().toggleOnTrue(m_automations.intakeCoralCommand());
        m_driver.R2().toggleOnTrue(m_automations.intakeAlgaeCommand());

        m_driver.options().onTrue(m_superstructure.collapseCommand());
        m_driver.create().onTrue(m_automations.cancelAutomationCommand());

        m_driver.R1().onTrue(m_automations.changeRightSlice());
        m_driver.L1().onTrue(m_automations.changeLeftSlice());

        m_operator.R1().onTrue(new InstantCommand(() -> this.right = true));
        m_operator.L1().onTrue(new InstantCommand(() -> this.right = false));

        m_operator.circle().onTrue(m_superstructure.ejectAlgaeCommand());
        m_operator.triangle().onTrue(m_superstructure.startAutomationCommand());
        m_operator.povDown().onTrue(m_superstructure.scoreAlgaeCommand(PROCESSOR_ID));

        m_operator.square().onTrue(m_automations.cancelAutomationCommand());

        m_operator.touchpad().whileTrue(m_superstructure.coastCommand().alongWith(m_swerve.coastCommand()).ignoringDisable(true));
    }


    public double deadband(double value) {
        return Math.abs(value) < DEADBAND_VALUE ? 0 : value;
    }

    private void initAutoChooser() {
        Command centerAutoCommand = new SequentialCommandGroup(
                m_automations.toggleAutoMode(),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_automations.L4Command(false),
                m_superstructure.collapseCommand()
        );

        Command leftAutoCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_swerve.resetOdometry(LEFT_AUTO_POSE.get())),
                m_automations.toggleAutoMode(),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_automations.L4Command(true),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_automations.intakeCoralCommand(),
                m_automations.toggleAutoMode(),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_automations.L4Command(false),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_superstructure.collapseCommand()
        );

        Command rightAutoCommand = new SequentialCommandGroup(
                new InstantCommand(() -> m_swerve.resetOdometry(RIGHT_AUTO_POSE.get())),
                m_automations.toggleAutoMode(),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_automations.L4Command(true),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_automations.intakeCoralCommand(),
                m_automations.toggleAutoMode(),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_automations.L4Command(false),
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_superstructure.collapseCommand()
        );

        m_autoChooser.setDefaultOption("empty auto", new InstantCommand());
        m_autoChooser.addOption("exit from line", m_swerve.driveCommand(() -> new Vector2D(1, 0), () -> 0, () -> false).withTimeout(2));
        m_autoChooser.addOption("center auto", centerAutoCommand);
        m_autoChooser.addOption("left auto", leftAutoCommand);
        m_autoChooser.addOption("right auto", rightAutoCommand);

        SmartDashboard.putData("autoChooser", m_autoChooser);
    }

    private void initElastic() {
        SmartDashboard.putNumber("match time", DriverStation.getMatchTime());
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

    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }


    @Log.NT
    public String getReefSlice() {
        return Slice.getSlice(m_swerve.getPose2D().getTranslation()).name();
    }

    @Log.NT
    public String getTragetSlice() {
        return m_automations.getTargetSlice().name();
    }

    @Log.NT
    public boolean getRightBranch() {
        return right;
    }

    @Log.NT
    public boolean getLeftBranch() {
        return !right;
    }

    @Log.NT
    public Pose2d intersections() {
        return new Pose2d(MathUtils.getTargetPose(m_swerve.getPose2D().getTranslation(), m_automations.getTargetSlice().generalPose.get().getTranslation(), getReefCenter()), new Rotation2d());
    }

    @Log.NT
    public boolean atAutoMode() {
        return m_automations.m_autoMode.getAsBoolean();
    }
}
