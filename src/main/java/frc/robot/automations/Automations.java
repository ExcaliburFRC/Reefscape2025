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
    private Trigger m_atTargetSlicePose;

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
        for (int i = 1; i < FEEDERS_POSES.length; i++) {
            if (distance > robot.getDistance(FEEDERS_POSES[i].get().getTranslation())) {
                feeder = FEEDERS_POSES[i];
                distance = robot.getDistance(feeder.get().getTranslation());
            }
        }
        return feeder;
    }

    public Command intakeCoralCommand() {
        return scheduleExclusiveCommand(new ParallelDeadlineGroup(
                m_superstructure.intakeCoralCommand(),
                m_swerve.pidToPoseCommand(() -> getCoralIntakePose().get())
        ).andThen(m_superstructure.collapseCommand()));
    }

    public Command intakeAlgaeCommand() {
        return new ConditionalCommand(
                new PrintCommand("has algae, cant intake one"),
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                m_superstructure.intakeAlgaeCommand(2),
                                m_superstructure.intakeAlgaeCommand(3),
                                () -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).ALGAE_LEVEL == 2
                        ),
                        m_swerve.pidToPoseCommand(() -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).generalPose.get()),
                        m_swerve.stopCommand().withTimeout(0.05),
                        m_swerve.pidToPoseCommand(() -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).alagePose.get()),
                        m_swerve.stopCommand().withTimeout(0.1),
                        new WaitUntilCommand(m_superstructure.hasAlgaeTrigger()),
                        m_swerve.pidToPoseCommand(() -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).generalPose.get()),
                        m_swerve.stopCommand().withTimeout(0.05),
                        m_superstructure.collapseCommand()
                ), m_superstructure.hasAlgaeTrigger()).alongWith(new PrintCommand(
                Slice.getSlice(m_swerve.getPose2D().getTranslation()).alagePose.toString()
        ));
    }

//    public Command scoreAlgaeCommand() {
//        return Commands.none();
//    }

    public Command L1Command() {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one"),
                        new SequentialCommandGroup(
                                m_leds.setPattern(LEDs.LEDPattern.SOLID, ORANGE.color).withDeadline(
                                        new SequentialCommandGroup(
                                                m_swerve.turnToAngleCommand(
                                                        () -> new Vector2D(0, 0),
                                                        () -> Slice.getSlice(
                                                                m_swerve.getPose2D().getTranslation()).l1Pose.get().getRotation()
                                                ).withTimeout(1),
                                                m_swerve.pidToPoseCommand(() -> Slice.getSlice(m_swerve.getPose2D().getTranslation()).l1Pose.get()),
                                                m_swerve.stopCommand().withTimeout(0.1),
                                                m_superstructure.alignToCoralCommand(1)
                                        )
                                ),
                                new WaitCommand(0.1),
                                scoreCoralCommand(),
                                new WaitCommand(3)

                        ), m_superstructure.hasCoralTrigger().negate())
        );
    }

    public Command L4Command(boolean right) {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one"),
                        new SequentialCommandGroup(

                                m_leds.setPattern(LEDs.LEDPattern.BLINKING, GREEN.color).withDeadline(
                                        new SequentialCommandGroup(
                                                m_superstructure.startAutomationCommand(),
                                                m_swerve.pidToPoseCommand(() -> Slice.getBranchPose(right, m_swerve.getPose2D().getTranslation()).get()),
                                                m_swerve.driveCommand(() -> new Vector2D(0.5, 0), () -> 0, () -> false).withTimeout(0.2),
                                                m_swerve.stopCommand().withTimeout(0.05),
                                                m_superstructure.alignToCoralCommand(4)
                                        )
                                ),
                                new WaitCommand(0.25),
                                scoreCoralCommand()

                        ), m_superstructure.hasCoralTrigger().negate())
        );
    }

    public Command L3Command(boolean right) {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one"),
                        new SequentialCommandGroup(
                                m_leds.setPattern(LEDs.LEDPattern.BLINKING, GREEN.color).withDeadline(
                                        new SequentialCommandGroup(
                                                m_superstructure.startAutomationCommand(),
                                                m_swerve.pidToPoseCommand(() -> Slice.getBranchPose(right, m_swerve.getPose2D().getTranslation()).get()),
                                                m_swerve.driveCommand(() -> new Vector2D(0.2, 0), () -> 0, () -> false).withTimeout(0.3),
                                                m_superstructure.alignToCoralCommand(3)
                                        )
                                ),
                                new WaitCommand(0.25),
                                scoreCoralCommand()
                        ),
                        m_superstructure.hasCoralTrigger().negate())
        );
    }

    public Command scoreCoralCommand() {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one"),
                        m_leds.setPattern(LEDs.LEDPattern.BLINKING, GREEN.color).withDeadline(
                                new SequentialCommandGroup(
                                        m_superstructure.scoreCoralCommand(),
                                        new ConditionalCommand(
                                                m_superstructure.collapseCommand(),
                                                m_superstructure.startAutomationCommand(),
                                                () -> m_superstructure.getState().equals(POST_L1) || m_superstructure.getState().equals(State.L1) || m_superstructure.getState().equals(State.DEFAULT)
                                        )
                                )
                        ),
                        m_superstructure.hasCoralTrigger().negate()
                ));
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
                () -> {
                }, // No execute action needed
                (interrupted) -> {
                }, // No special end behavior needed
                () -> !m_runningCommand.isScheduled() // Ends when the command is no longer scheduled
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
                                ).generalPose.get().getRotation()
                        )
                ).until(m_atTargetSlicePose),
                m_swerve.stopCommand().until(m_atTargetSlicePose.negate())
        );
    }

    private boolean poseAtTolerance(Pose2d pose1, Pose2d pose2) {
        Translation2d delta = pose1.minus(pose2).getTranslation();
        Rotation2d deltaAngle = pose1.getRotation().minus(pose2.getRotation());
        return (Math.abs(delta.getX()) < x_TOLERANCE) &&
                (Math.abs(delta.getY()) < Y_TOLERANCE) &&
                (Math.abs(deltaAngle.getRadians()) < ANGLE_TOLERANCE);
    }
}
