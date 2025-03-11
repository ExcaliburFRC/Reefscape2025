package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.Superstructure;
import monologue.Annotations;
import monologue.Annotations.Log;
import monologue.Logged;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.NET_ID;

public class Automations {
    Superstructure m_superstructure;
    Swerve m_swerve;

    public Automations(Swerve swerve, Superstructure superstructure) {
        this.m_superstructure = superstructure;
        this.m_swerve = swerve;
    }

    @Log.NT
    public int getReefSlice() {
        Translation2d robotTranslation = m_swerve.getPose2D().getTranslation();
        robotTranslation = robotTranslation.minus(Constants.FieldConstants.REEF_CENTER);
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

    private Pose2d getCoralIntakePose(boolean right) {
        return m_swerve.getPose2D();
    }

    private Pose2d getAlgaeIntakePose() {
        if (getReefSlice() == -1) {
            System.out.println("invalid angle");
            return m_swerve.getPose2D();
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


    private Pose2d getAlgaeIntakePostPose() {
        if (getReefSlice() == -1) {
            System.out.println("invalid angle");
            return m_swerve.getPose2D();
        }
        return POST_ALGAES[getReefSlice()];
    }

    private Pose2d getCoralScorePose(int level, boolean right) {
        if (getReefSlice() == -1) {
            System.out.println("invalid angle");
            return m_swerve.getPose2D();
        }
        if (level == 1) {
            return L1s[getReefSlice()];
        }
        if (right) {
            return RIGHT_BRANCHES[getReefSlice()];
        }
        return LEFT_BRANCHES[getReefSlice()];
    }


    public Command intakeCoralCommand(boolean right) {
        return new ParallelDeadlineGroup(
                m_superstructure.intakeCoralCommand(),
                m_swerve.pidToPoseCommand(() -> getCoralIntakePose(right))
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
                        m_swerve.pidToPoseCommand(this::getAlgaeIntakePose),
                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.01),
                        new WaitUntilCommand(m_superstructure.hasAlgaeTrigger()),
                        m_swerve.pidToPoseCommand(this::getAlgaeIntakePostPose),
                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.01),
                        m_superstructure.collapseCommand()
                ), m_superstructure.hasAlgaeTrigger());
    }

    public Command scoreAlgaeCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        m_swerve.pidToPoseCommand(()->new Pose2d()),
                        m_superstructure.alignToAlgaeCommand(NET_ID),
                        m_superstructure.scoreAlgaeCommand(NET_ID),
                        m_superstructure.startAutomationCommand(),
                        m_superstructure.collapseCommand()
                ),
                new PrintCommand("has no algae"),
                m_superstructure.hasAlgaeTrigger()
        );
    }

    public Command L1Command(boolean right) {
        return new ConditionalCommand(
                new PrintCommand("doesn't have coral, cant score one"),
                new SequentialCommandGroup(
                        m_swerve.pidToPoseCommand(() -> getCoralScorePose(1, false)),
                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.1),
                        m_superstructure.alignToCoralCommand(1),
                        new WaitCommand(1),
                        m_superstructure.scoreCoralCommand(1)
                ), m_superstructure.hasCoralTrigger().negate());
    }

    public Command L4Command(boolean right) {
        return new ConditionalCommand(
                new PrintCommand("doesn't have coral, cant score one"),
                new SequentialCommandGroup(
                        m_superstructure.startAutomationCommand(),
                        m_swerve.pidToPoseCommand(() -> getCoralScorePose(4, right)),
                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.1),
                        m_superstructure.alignToCoralCommand(4),
                        new WaitCommand(1),
                        m_superstructure.scoreCoralCommand(4),
                        m_superstructure.startAutomationCommand()
                ), m_superstructure.hasCoralTrigger().negate());
    }

    public Command L3Command(boolean right) {
        return new ConditionalCommand(
                new PrintCommand("doesn't have coral, cant score one"),
                new SequentialCommandGroup(
                        m_superstructure.startAutomationCommand(),
                        m_swerve.pidToPoseCommand(() -> getCoralScorePose(3, right)),
                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.1),
                        m_superstructure.alignToCoralCommand(3),
                        new WaitCommand(0.5),
                        m_superstructure.scoreCoralCommand(3),
                        m_superstructure.startAutomationCommand()
                ), m_superstructure.hasCoralTrigger().negate());
    }

    public Command L2Command(boolean right) {
        return new ConditionalCommand(
                new PrintCommand("doesn't have coral, cant score one"),
                new SequentialCommandGroup(
                        m_superstructure.alignToCoralCommand(2),
                        m_swerve.pidToPoseCommand(() -> getCoralScorePose(2, true)),
                        m_swerve.driveCommand(() -> new Vector2D(0, 0), () -> 0, () -> true).withTimeout(0.1),
                        new WaitCommand(1),
                        new PrintCommand("whhhhhaa"),
                        m_superstructure.scoreCoralCommand(2),
                        m_superstructure.startAutomationCommand()
                ), m_superstructure.hasCoralTrigger().negate());
    }
}