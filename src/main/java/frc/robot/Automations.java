package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.Superstructure;

public class Automations {
    Superstructure m_superstructure;
    Swerve m_swerve;

    public Automations(Swerve swerve, Superstructure superstructure) {
        this.m_superstructure = superstructure;
        this.m_swerve = swerve;
    }

    private Pose2d getCoralIntakePose(boolean right) {
        return m_swerve.getPose2D();
    }

    private Pose2d getAlgaeIntakePose() {
        return m_swerve.getPose2D();
    }

    private int getAlgaeIntakeLevel() {
        return 2;
    }


    private Pose2d getAlgaeIntakePrePose() {
        return m_swerve.getPose2D();
    }

    private Pose2d getCoralScorePose(int level, boolean right) {
        return Constants.FieldConstants.B5.pose;
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
                        m_swerve.pidToPoseCommand(this::getAlgaeIntakePrePose),
                        m_superstructure.intakeAlgaeCommand(getAlgaeIntakeLevel()),
                        m_swerve.pidToPoseCommand(this::getAlgaeIntakePose),
                        m_swerve.pidToPoseCommand(this::getAlgaeIntakePrePose),
                        m_superstructure.collapseCommand()
                ), m_superstructure.hasAlgaeTrigger());
    }

    public Command scoreCoralCommand(int level, boolean right) {
        return new ConditionalCommand(
                new PrintCommand("doesn't have coral, cant score one"),
                new SequentialCommandGroup(
                        m_superstructure.startAutomationCommand(),
                        m_swerve.pidToPoseCommand(() -> getCoralScorePose(level, false)),
                        m_swerve.driveCommand(()->new Vector2D(0,0),()->0,()->true).withTimeout(0.1),
                        m_superstructure.alignToCoralCommand(level),
                        new WaitCommand(1),
                        m_superstructure.scoreCoralCommand(level),
                        m_superstructure.startAutomationCommand()
                ), m_superstructure.hasCoralTrigger().negate());
    }
}