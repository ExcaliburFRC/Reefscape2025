package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.LEDs;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.State;
import frc.robot.superstructure.Superstructure;
import monologue.Annotations.Log;

import static frc.excalib.additional_utilities.AllianceUtils.*;
import static frc.excalib.additional_utilities.Color.Colors.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.superstructure.State.INTAKE;
import static frc.robot.superstructure.State.POST_L1;

public class Automations {
    Superstructure m_superstructure;
    Swerve m_swerve;
    private Command m_runningCommand;
    private final LEDs m_leds = LEDs.getInstance();

    public Automations(Swerve swerve, Superstructure superstructure) {
        this.m_superstructure = superstructure;
        this.m_swerve = swerve;
        this.m_runningCommand = null;
    }

    @Log.NT
    public int getReefSlice() {
        Translation2d robotTranslation = m_swerve.getPose2D().getTranslation();
        if (AllianceUtils.isRedAlliance()) {
            robotTranslation = new Translation2d(FIELD_LENGTH_METERS - robotTranslation.getX(), FIELD_WIDTH_METERS - robotTranslation.getY());
        }
        robotTranslation = robotTranslation.minus(Constants.FieldConstants.BLUE_REEF_CENTER);
        double angle = robotTranslation.getAngle().getDegrees();
        if (angle < 30 && angle > -30) {
            return 0;
        }
        if (angle < -30 && angle > -90) {
            return 1;
        }
        if (angle < -90 && angle > -150) {
            return 2;
        }
        if (angle > 150 || angle < -150) {
            return 3;
        }
        if (angle > 90 && angle < 150) {
            return 4;
        }
        if (angle > 30 && angle < 90) {
            return 5;
        }
        return -1;
    }

    private AllianceUtils.AlliancePose getCoralIntakePose(boolean right) {
        if (isRedAlliance())
            if (m_swerve.getPose2D().getY() < FIELD_WIDTH_METERS / 2) {
                return FEADERS_POSES[0];
            }
        return FEADERS_POSES[1];
    }

    private AllianceUtils.AlliancePose getAlgaeIntakePose() {
        if (getReefSlice() == -1) {
            System.out.println("invalid angle");
            return new AllianceUtils.AlliancePose(
                    m_swerve.getPose2D().getTranslation(),
                    new Rotation2d(m_swerve.getPose2D().getRotation().getDegrees()
                    )
            );
        }
        return ALGAES[getReefSlice()];
    }

    @Log.NT
    private int getAlgaeIntakeLevel() {
        switch (getReefSlice()) {
            case 0, 2, 4 -> {
                return 2;
            }
            case 1, 3, 5 -> {
                return 3;
            }
            default -> {
                return -1;
            }
        }
    }

    private AllianceUtils.AlliancePose getAlgaeIntakePostPose() {
        if (getReefSlice() == -1) {
            System.out.println("invalid angle");
            return new AllianceUtils.AlliancePose(m_swerve.getPose2D().getTranslation(), m_swerve.getRotation2D());
        }
        return POST_ALGAES[getReefSlice()];
    }

    private Pose2d getCoralScorePose(int level, boolean right) {
        if (getReefSlice() == -1) {
            System.out.println("invalid angle");
            return m_swerve.getPose2D();
        }
        if (level == 1) {
            return L1s[getReefSlice()].get();
        }
        if (right) {
            return RIGHT_BRANCHES[getReefSlice()].get();
        }
        return LEFT_BRANCHES[getReefSlice()].get();
    }


    public Command intakeCoralCommand(boolean right) {
        return new ParallelDeadlineGroup(
                m_superstructure.intakeCoralCommand(),
                m_swerve.pidToPoseCommand(() -> getCoralIntakePose(right).get())
        ).andThen(m_superstructure.collapseCommand());
    }

    public Command intakeAlgaeCommand() {
        return new ConditionalCommand(
                new PrintCommand("has algae, cant intake one"),
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                m_superstructure.intakeAlgaeCommand(2),
                                m_superstructure.intakeAlgaeCommand(3),
                                () -> getAlgaeIntakeLevel() == 2
                        ),
                        m_swerve.pidToPoseCommand(() -> getAlgaeIntakePostPose().get()),
                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.05),
                        m_swerve.pidToPoseCommand(() -> getAlgaeIntakePose().get()),
                        m_swerve.driveCommand(() -> new Vector2D(1, 0), () -> 0, () -> false).withTimeout(0.1),
                        new WaitUntilCommand(m_superstructure.hasAlgaeTrigger()),
                        m_swerve.pidToPoseCommand(() -> getAlgaeIntakePostPose().get()),
                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.05),
                        m_superstructure.collapseCommand()
                ), m_superstructure.hasAlgaeTrigger()).alongWith(new PrintCommand(
                getAlgaeIntakePose().toString()
        ));
    }

//    public Command scoreAlgaeCommand() {
//        return Commands.none();
//    }

    public Command alignToL1Command() {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one"),
                        m_leds.setPattern(LEDs.LEDPattern.SOLID, ORANGE.color).withDeadline(
                                new SequentialCommandGroup(
                                        m_swerve.pidToPoseCommand(() -> getCoralScorePose(1, false)),
                                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.1),
                                        m_superstructure.alignToCoralCommand(1)
                                )
                        ), m_superstructure.hasCoralTrigger().negate())
        );
    }

    public Command alignToL4Command(boolean right) {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one"),
                        m_leds.setPattern(LEDs.LEDPattern.BLINKING, GREEN.color).withDeadline(
                                new SequentialCommandGroup(
                                        m_superstructure.startAutomationCommand().alongWith(
                                                m_swerve.pidToPoseCommand(() -> getCoralScorePose(4, right))),
                                        m_swerve.driveCommand(() -> new Vector2D(0.5, 0), () -> 0, () -> false).withTimeout(0.2),
                                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> false).withTimeout(0.05),
                                        m_superstructure.alignToCoralCommand(4)
                                )
                        ), m_superstructure.hasCoralTrigger().negate())
        );
    }

    public Command alignToL3Command(boolean right) {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new PrintCommand("doesn't have coral, cant score one"),
                        m_leds.setPattern(LEDs.LEDPattern.BLINKING, GREEN.color).withDeadline(new SequentialCommandGroup(
                                        m_superstructure.startAutomationCommand().alongWith(m_swerve.pidToPoseCommand(() -> getCoralScorePose(3, right))),
                                        m_swerve.driveCommand(() -> new Vector2D(0.5, 0), () -> 0, () -> false).withTimeout(0.3),
                                        m_superstructure.alignToCoralCommand(3)
                                )
                        ), m_superstructure.hasCoralTrigger().negate())
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

}
