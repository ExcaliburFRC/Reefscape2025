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
    //    private final CommandPS5Controller m_operator = new CommandPS5Controller(1);
    private final InterpolatingDoubleTreeMap m_decelerator = new InterpolatingDoubleTreeMap();
    private final Automations m_automations;
    private boolean autoMode = false;


    private SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        m_decelerator.put(-1.0, 1.0);
        m_decelerator.put(1.0, 0.25);
        m_automations = new Automations(m_swerve, m_superstructure);


//        initAutoChooser();
        initElastic();

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {

        m_swerve.setDefaultCommand(
                new SequentialCommandGroup(
                        m_automations.autoSwerveCommand().until(() -> !autoMode),
                        m_swerve.driveCommand(
                                () -> new Vector2D(
                                        deadband(-m_driver.getLeftY()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3)),
                                        deadband(-m_driver.getLeftX()) * MAX_VEL * m_decelerator.get(m_driver.getRawAxis(3))),
                                () -> deadband(-m_driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC * (!m_driver.L2().getAsBoolean() ? m_decelerator.get(m_driver.getRawAxis(3)) : 0.065),
                                () -> true
                        ).until(() -> autoMode)
                )
        );

        m_driver.PS().onTrue(m_swerve.resetAngleCommand());

//        m_driver.cross().toggleOnTrue(m_automations.L1Command());
//        m_driver.povDown().toggleOnTrue(m_automations.L1Command());

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


        m_driver.povRight().onTrue(new InstantCommand(() -> autoMode = !autoMode));
        m_driver.povUp().toggleOnTrue(m_automations.intakeCoralCommand());
        m_driver.R2().toggleOnTrue(m_automations.intakeAlgaeCommand());

        m_driver.options().onTrue(m_superstructure.collapseCommand());

        m_driver.R1().onTrue(m_automations.changeRightSlice());
        m_driver.L1().onTrue(m_automations.changeLeftSlice());


//
//        m_operator.R1().onTrue(new InstantCommand(() -> this.right = true));
//        m_operator.L1().onTrue(new InstantCommand(() -> this.right = false));
//
//        m_operator.circle().onTrue(m_superstructure.ejectAlgaeCommand());
//        m_operator.triangle().onTrue(m_superstructure.startAutomationCommand());
//        m_operator.povDown().onTrue(m_superstructure.scoreAlgaeCommand(PROCESSOR_ID));

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
        return Commands.none();
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
    public String getRightBranch() {
        return right ? "RIGHT" : "LEFT";
    }

    @Log.NT
    public Pose2d intersections() {
        return new Pose2d(MathUtils.getTargetPose(m_swerve.getPose2D().getTranslation(), m_automations.getTargetSlice().generalPose.get().getTranslation(), getReefCenter()), new Rotation2d());
    }

    @Log.NT
    public boolean isAutoMode() {
        return autoMode;
    }
}
