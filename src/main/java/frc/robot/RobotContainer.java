// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.excalib.commands.MapCommand;
import frc.excalib.control.math.MathUtils;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.automations.Automations;
import frc.robot.automations.IntakeState;
import frc.robot.automations.ScoreState;
import frc.robot.automations.Slice;
import frc.robot.superstructure.Superstructure;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.automations.Constants.FieldConstants.getReefCenter;
import static frc.robot.automations.ScoreState.*;
import static frc.robot.automations.ScoreState.EJECT_ALGAE;
import static frc.robot.automations.ScoreState.L1;

public class RobotContainer implements Logged {
    private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

    private final Swerve m_swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());
    private final Superstructure m_superstructure = new Superstructure();
    private final LEDs leds = LEDs.getInstance();
    private boolean right;
    private ScoreState m_currentScoreState = null;
    private IntakeState m_currentIntakeState = null;

    public final Runnable m_odometryUpdater = m_swerve::updateOdometry;

    private final CommandPS5Controller m_driver = new CommandPS5Controller(0);
    private final CommandPS5Controller m_operator = new CommandPS5Controller(1);
    private final InterpolatingDoubleTreeMap m_decelerator = new InterpolatingDoubleTreeMap();
    private final Automations m_automations;

    private final Trigger m_deportAutoMode = new Trigger(() -> Math.sqrt(
            Math.pow(m_driver.getLeftX(), 2) + Math.pow(m_driver.getLeftY(), 2)) > 0.2);

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
        Map<IntakeState, Command> intakeMap = new HashMap();
        Map<ScoreState, Command> scoreMap = new HashMap();

        scoreMap.put(L4_RIGHT, m_automations.L4Command(true));
        scoreMap.put(L4_LEFT, m_automations.L4Command(false));
        scoreMap.put(L3_RIGHT, m_automations.L3Command(true));
        scoreMap.put(L3_LEFT, m_automations.L3Command(false));
        scoreMap.put(L1, m_automations.L1Command());
        scoreMap.put(EJECT_ALGAE, m_automations.ejectAlgaeCommand());
        scoreMap.put(NET, m_automations.netCommand());
        scoreMap.put(null, new PrintCommand("Score Mode Not Selected!"));

        intakeMap.put(IntakeState.CORAL, m_automations.manualIntakeCommand());
        intakeMap.put(IntakeState.ALGAE, m_automations.intakeAlgaeCommand());
        intakeMap.put(IntakeState.AUTO_CORAL, m_automations.intakeCoralCommand());
        intakeMap.put(null, new PrintCommand("Intake Mode Not Selected!"));

        m_swerve.setDefaultCommand(
                new SequentialCommandGroup(
                        m_automations.autoSwerveCommand().until(m_automations.m_autoMode.negate()),
                        m_swerve.driveCommand(
                                () -> new Vector2D(
                                        deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
                                        deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
                                () -> deadband(-m_driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC * m_decelerator.get(m_driver.getRawAxis(3)),
                                () -> true
                        ).until(m_automations.m_autoMode)
                )
        );

        m_operator.cross().toggleOnTrue(
                new InstantCommand(
                        () -> this.m_currentScoreState = L1
                )
        );

        m_operator.circle().toggleOnTrue(
                new InstantCommand(
                        () -> {
                            if (right) {
                                this.m_currentScoreState = L3_RIGHT;
                            } else {
                                this.m_currentScoreState = L3_LEFT;
                            }
                        }
                )
        );

        m_operator.triangle().toggleOnTrue(
                new InstantCommand(
                        () -> {
                            if (right) {
                                this.m_currentScoreState = L4_RIGHT;
                            } else {
                                this.m_currentScoreState = L4_LEFT;
                            }
                        }
                )
        );

        m_operator.R1().onTrue(
                new InstantCommand(
                        () -> {
                            this.right = true;
                            if (m_currentScoreState.equals(L4_LEFT)) {
                                m_currentScoreState = L4_RIGHT;
                            }
                            if (m_currentScoreState.equals(L3_LEFT)) {
                                m_currentScoreState = L3_RIGHT;
                            }
                        }
                ).ignoringDisable(true));

        m_operator.L1().onTrue(
                new InstantCommand(
                        () -> {
                            this.right = false;
                            if (m_currentScoreState.equals(L4_RIGHT)) {
                                m_currentScoreState = L4_LEFT;
                            }
                            if (m_currentScoreState.equals(L3_RIGHT)) {
                                m_currentScoreState = L3_LEFT;
                            }
                        }).ignoringDisable(true));

        m_operator.povUp().toggleOnTrue(new InstantCommand(() -> this.m_currentScoreState = NET));

        m_operator.R2().onTrue(new InstantCommand(() -> m_currentIntakeState = IntakeState.AUTO_CORAL));
        m_operator.L2().onTrue(new InstantCommand(() -> m_currentIntakeState = IntakeState.ALGAE));
        m_operator.povDown().onTrue(new InstantCommand(() -> m_currentIntakeState = IntakeState.CORAL));

        m_operator.povLeft().onTrue(new InstantCommand(() -> m_currentScoreState = EJECT_ALGAE));

        m_driver.R1().onTrue(m_automations.changeRightSlice());
        m_driver.L1().onTrue(m_automations.changeLeftSlice());

        m_driver.triangle().onTrue(new MapCommand<>(scoreMap, () -> m_currentScoreState));
        m_driver.povUp().onTrue(new MapCommand<>(intakeMap, () -> m_currentIntakeState));

        m_driver.povDown().onTrue(m_automations.toggleAutoMode());
        m_deportAutoMode.onTrue(m_automations.setAutoMode(() -> false));

        m_operator.options().onTrue(m_automations.collapseCommand());
        m_operator.create().onTrue(m_automations.cancelAutomationCommand());

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
                new WaitUntilCommand(m_automations.m_atTargetSlicePose),
                m_automations.intakeAlgaeCommand(),
                m_automations.toggleAutoMode(),
                m_automations.netCommand()

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

        m_autoChooser.setDefaultOption("Empty Automation", new InstantCommand());
        m_autoChooser.addOption("Exit From Start Line", m_swerve.driveCommand(() -> new Vector2D(1, 0), () -> 0, () -> false).withTimeout(2));
        m_autoChooser.addOption("Center Automation", centerAutoCommand);
        m_autoChooser.addOption("Left Automation", leftAutoCommand);
        m_autoChooser.addOption("Right Automation", rightAutoCommand);

        SmartDashboard.putData("autoChooser", m_autoChooser);
    }

    private void initElastic() {
        SmartDashboard.putNumber("match time", DriverStation.getMatchTime());
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

    @Log.NT
    public String getIntakeMode(){
    return m_currentIntakeState != null ? m_currentIntakeState.name(): "null";
    }

    @Log.NT
    public String getScoreMode(){
        return m_currentScoreState != null? m_currentScoreState.name() : "null";
    }
}
