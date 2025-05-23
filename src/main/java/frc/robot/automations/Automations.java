package frc.robot.automations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.control.math.MathUtils;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.State;
import frc.robot.superstructure.Superstructure;

import java.util.function.BooleanSupplier;

import static frc.excalib.additional_utilities.AllianceUtils.*;
import static frc.excalib.additional_utilities.Color.Colors.*;
import static frc.robot.automations.Constants.FieldConstants.*;
import static frc.robot.superstructure.State.POST_L1;

public class Automations {
    Superstructure m_superstructure;
    Swerve m_swerve;
    private Command m_runningCommand;
    private Slice m_tragetSlice = Slice.SECOND;
    private final LEDs m_leds = LEDs.getInstance();
    public final Trigger m_atTargetSlicePose;
    private boolean autoMode = false;
    public final Trigger m_autoMode = new Trigger(() -> autoMode);
    private final Trigger atNetPoseTrigger = new Trigger(() -> Math.abs(m_swerve.getPose2D().getX() - NET_X_VALUE) < 0.03);

    public Automations(Swerve swerve, Superstructure superstructure) {
        this.m_superstructure = superstructure;
        this.m_swerve = swerve;
        this.m_runningCommand = null;
        this.m_atTargetSlicePose = new Trigger(() -> poseAtTolerance(m_swerve.getPose2D(), m_tragetSlice.generalPose.get()));
    }

    private AlliancePose getCoralIntakePose() {
        Translation2d robot = m_swerve.getPose2D().getTranslation();
        AlliancePose feeder = FEEDERS_POSES[0];
        double distance = feeder.get().getTranslation().getDistance(robot);
            for (AlliancePose selectedPose : FEEDERS_POSES) {
            if (distance > robot.getDistance(selectedPose.get().getTranslation())) {
                feeder = selectedPose;
                distance = robot.getDistance(feeder.get().getTranslation());
            }
        }
        return feeder;
    }

    private Pose2d getNetPose() {
        Translation2d robot = m_swerve.getPose2D().getTranslation();
        AlliancePose netPose = NET_POSES[0];
        double distance = netPose.get().getTranslation().getDistance(robot);
        for (AlliancePose selectedPose : NET_POSES) {
            if (distance > robot.getDistance(netPose.get().getTranslation())) {
                netPose = selectedPose;
                distance = robot.getDistance(netPose.get().getTranslation());
            }
        }
        return netPose.get();
    }

    private Pose2d getNetPosePose() {
        Translation2d robot = m_swerve.getPose2D().getTranslation();
        AlliancePose netPose = POST_NET_POSES[0];
        double distance = netPose.get().getTranslation().getDistance(robot);
        for (AlliancePose selectedPose : POST_NET_POSES){
            if (distance > robot.getDistance(selectedPose.get().getTranslation())) {
                netPose = selectedPose;
                distance = robot.getDistance(netPose.get().getTranslation());
            }
        }
        return netPose.get();
    }

    public Command intakeCoralCommand() {
        return scheduleExclusiveCommand(
                new ParallelDeadlineGroup(
                        m_superstructure.intakeCoralCommand(),
                        setAutoMode(() -> false),
                        m_swerve.pidToPoseCommand(() -> getCoralIntakePose().get())
                ).andThen(m_superstructure.collapseCommand())
        );
    }

    public Command intakeAlgaeCommand() {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("has algae, cant intake one").andThen(setAutoMode(() -> true)),
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        m_superstructure.intakeAlgaeCommand(2),
                                        m_superstructure.intakeAlgaeCommand(3),
                                        () -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).ALGAE_LEVEL == 2
                                ),
                                m_swerve.pidToPoseCommand(
                                        () -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).alagePose.get()
                                ).until(m_superstructure.hasAlgaeTrigger()),
                                new WaitUntilCommand(m_superstructure.hasAlgaeTrigger()),
                                m_swerve.pidToPoseCommand(
                                        () -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).generalPose.get()),
                                m_swerve.stopCommand().withTimeout(0.05)
                        ), m_superstructure.hasAlgaeTrigger().or(
                        m_autoMode.negate()).or(m_atTargetSlicePose.negate())
                )
        );
    }

    public Command L1Command(Trigger overrideTrigger) {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one"),
                        new SequentialCommandGroup(
                                m_leds.setPattern(LEDs.LEDPattern.TRAIN_CIRCLE, PURPLE.color, WHITE.color).withDeadline(
                                        new SequentialCommandGroup(
                                                m_superstructure.collapseCommand(),
                                                m_swerve.pidToPoseCommand(() -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).l1Pose.get()),
                                                m_swerve.driveCommand(() -> new Vector2D(-0.5, 0), () -> 0, () -> false).withTimeout(0.2),
                                                m_swerve.stopCommand().withTimeout(0.05),
                                                m_superstructure.alignToCoralCommand(1)
                                        )
                                ),
                                scoreCoralCommand(),
                                new WaitCommand(0.3),
                                m_superstructure.collapseCommand()
                        ),
                        m_superstructure.hasCoralTrigger().negate()
                ).until(overrideTrigger)
        );
    }

    public Command L4Command(boolean right) {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one").andThen(setAutoMode(() -> true)),
                        new SequentialCommandGroup(
                                m_leds.setPattern(LEDs.LEDPattern.TRAIN_CIRCLE, PURPLE.color, WHITE.color).withDeadline(
                                        new SequentialCommandGroup(
                                                m_superstructure.startAutomationCommand(),
                                                m_superstructure.alignToCoralCommand(4).alongWith(
                                                        new SequentialCommandGroup(
                                                                m_swerve.pidToPoseCommand(() -> Slice.getBranchPose(right, m_swerve.getPose2D().getTranslation()).get()),
                                                                m_swerve.driveCommand(() -> new Vector2D(0.5, 0), () -> 0, () -> false).withTimeout(0.2),
                                                                m_swerve.stopCommand().withTimeout(0.05)
                                                        )
                                                )
                                        )
                                ),
                                new WaitCommand(0.6),
                                scoreCoralCommand(),
                                resetSuperstructureCommand().alongWith(
                                        m_swerve.pidToPoseCommand(
                                                () -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).generalPose.get()
                                        ).until(m_atTargetSlicePose)
                                )
                        ), m_superstructure.hasCoralTrigger().negate().or(
                        m_autoMode.negate()).or(
                        m_atTargetSlicePose.negate())
                )
        );
    }

    public Command L3Command(boolean right) {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one").andThen(setAutoMode(() -> true)),
                        new SequentialCommandGroup(
                                m_leds.setPattern(LEDs.LEDPattern.TRAIN_CIRCLE, PURPLE.color, WHITE.color).withDeadline(
                                        new SequentialCommandGroup(
                                                m_superstructure.startAutomationCommand(),
                                                m_superstructure.alignToCoralCommand(3).alongWith(
                                                        new SequentialCommandGroup(
                                                                m_swerve.pidToPoseCommand(() -> Slice.getBranchPose(right, m_swerve.getPose2D().getTranslation()).get()),
                                                                m_swerve.driveCommand(() -> new Vector2D(0.2, 0), () -> 0, () -> false).withTimeout(0.2),
                                                                m_swerve.stopCommand().withTimeout(0.05)
                                                        )
                                                )
                                        )
                                ),
                                new WaitCommand(0.5),
                                scoreCoralCommand(),
                                resetSuperstructureCommand().alongWith(
                                        m_swerve.pidToPoseCommand(
                                                () -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).generalPose.get()
                                        ).until(m_atTargetSlicePose)
                                )
                        ),
                        m_superstructure.hasCoralTrigger().negate().or(
                                m_autoMode.negate()).or(
                                m_atTargetSlicePose.negate()
                        )
                )
        );
    }

    public Command scoreCoralCommand() {
        return new ConditionalCommand(
                new PrintCommand("doesn't have coral, cant score one"),
                m_leds.setPattern(LEDs.LEDPattern.BLINKING, GREEN.color).withDeadline(m_superstructure.scoreCoralCommand()),
                m_superstructure.hasCoralTrigger().negate()
        );
    }

    private Command scheduleExclusiveCommand(Command newCommand) {
        return new FunctionalCommand(
                () -> {
                    if (m_runningCommand != null && m_runningCommand.isScheduled()) {
                        m_runningCommand.cancel();
                    }
                    m_runningCommand = newCommand;
                    m_runningCommand.schedule();
                },
                () -> {},
                (interrupted) -> {},
                () -> !m_runningCommand.isScheduled()
        );
    }

    private void changeSlice(boolean right) {
        switch (m_tragetSlice) {
            case FIRST -> m_tragetSlice = right ? Slice.SIXTH : Slice.SECOND;
            case SECOND -> m_tragetSlice = right ? Slice.FIRST : Slice.THIRD;
            case THIRD -> m_tragetSlice = right ? Slice.SECOND : Slice.FOURTH;
            case FOURTH -> m_tragetSlice = right ? Slice.THIRD : Slice.FIFTH;
            case FIFTH -> m_tragetSlice = right ? Slice.FOURTH : Slice.SIXTH;
            case SIXTH -> m_tragetSlice = right ? Slice.FIFTH : Slice.FIRST;
        }
    }

    public Command changeRightSlice() {
        return new InstantCommand(() -> changeSlice(true)).ignoringDisable(true);
    }

    public Command changeLeftSlice() {
        return new InstantCommand(() -> changeSlice(false)).ignoringDisable(true);
    }

    public Slice getTargetSlice() {
        return m_tragetSlice;
    }

    public Command autoSwerveCommand() {
        return new SequentialCommandGroup(
                m_swerve.pidToPoseCommand(
                        () -> new Pose2d(
                                MathUtils.getTargetPose(
                                        m_swerve.getPose2D().getTranslation(),
                                        getTargetSlice().generalPose.get().getTranslation(),
                                        getReefCenter()
                                ),
                                Slice.getSlice(
                                        m_swerve.getPose2D().getTranslation()
                                ).equals(m_tragetSlice) ? m_tragetSlice.generalPose.get().getRotation() :
                                        getReefCenter().minus(m_swerve.getPose2D().getTranslation()).getAngle()
                        )
                ).until(m_atTargetSlicePose),
                m_swerve.stopCommand().until(m_atTargetSlicePose.negate())
        ).alongWith(
                new ConditionalCommand(
                        new ConditionalCommand(
                                new InstantCommand(),
                                m_superstructure.startAutomationCommand(),
                                () -> m_superstructure.getState().equals(POST_L1) || m_superstructure.getState().equals(State.L1)
                        ),
                        new InstantCommand(),
                        m_autoMode
                )
        );
    }

    private boolean poseAtTolerance(Pose2d pose1, Pose2d pose2) {
        Translation2d delta = pose1.minus(pose2).getTranslation();
        Rotation2d deltaAngle = pose1.getRotation().minus(pose2.getRotation());
        return (Math.abs(delta.getX()) < x_TOLERANCE) &&
                (Math.abs(delta.getY()) < Y_TOLERANCE) &&
                (Math.abs(deltaAngle.getRadians()) < ANGLE_TOLERANCE);
    }

    public Command toggleAutoMode() {
        return new InstantCommand(() -> {
            autoMode = !autoMode;
            if (autoMode) m_tragetSlice = Slice.getSlice(m_swerve.getPose2D().getTranslation());
        });
    }

    private Command resetSuperstructureCommand() {
        return new ConditionalCommand(
                m_superstructure.collapseCommand(),
                m_superstructure.startAutomationCommand(),
                () -> m_superstructure.getState().equals(POST_L1) ||
                        m_superstructure.getState().equals(State.L1) ||
                        m_superstructure.getState().equals(State.DEFAULT)
        );
    }

    public Command cancelAutomationCommand() {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        m_superstructure.collapseCommand(),
                        new InstantCommand(),
                        () -> m_superstructure.getState().equals(State.INTAKE)
                )
        );
    }

    public boolean atNetPose() {
        return atNetPoseTrigger.getAsBoolean();
    }

    public Command netCommand() {
        return scheduleExclusiveCommand(
                new SequentialCommandGroup(
                        setAutoMode(() -> false),
                        m_swerve.pidToPoseCommand(this::getNetPosePose),
                        m_superstructure.alignToAlgaeCommand(4),
                        m_swerve.pidToPoseCommand(this::getNetPose),
                        m_superstructure.scoreAlgaeCommand(4),
                        m_swerve.pidToPoseCommand(this::getNetPosePose),
                        m_superstructure.collapseCommand().alongWith(
                                m_swerve.driveCommand(
                                        () -> new Vector2D(-0.7, 0),
                                        () -> 0,
                                        () -> false
                                ).withTimeout(2)
                        )
                )
        );
    }

    public Command manualIntakeCommand() {
        return scheduleExclusiveCommand(
                new SequentialCommandGroup(
                        setAutoMode(() -> false),
                        m_superstructure.intakeCoralCommand(),
                        m_superstructure.collapseCommand()
                )
        );
    }

    public Command collapseCommand() {
        return scheduleExclusiveCommand(m_superstructure.collapseCommand());
    }

    public Command setAutoMode(BooleanSupplier mode) {
        return new InstantCommand(() -> {
            autoMode = mode.getAsBoolean();
            if (autoMode) m_tragetSlice = Slice.getSlice(
                    m_swerve.getPose2D().getTranslation()
            );
        });
    }

    public Command ejectAlgaeCommand() {
        return scheduleExclusiveCommand(m_superstructure.ejectAlgaeCommand());
    }

    public Command forceShootCoral() {
        return scheduleExclusiveCommand(m_superstructure.forceShootCoral());
    }
}

