package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algae.AlgaeSystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;

public class Superstructure implements Logged {
    private Elevator m_elevator;
    private Arm m_arm;
    private CoralSystem m_coralSystem;
    private AlgaeSystem m_algaeSystem;
    public Trigger m_toleranceTrigger;
    private State m_currentState;

    public Superstructure() {
        m_elevator = new Elevator();
        m_arm = new Arm();
        m_algaeSystem = new AlgaeSystem();
        m_coralSystem = new CoralSystem();

        m_arm.setElevatorHeightSupplier(m_elevator.m_heightSupplier);
        m_arm.setHasAlgaeTrigger(() -> m_algaeSystem.hasAlgae());
        m_arm.setHasCoralTrigger(m_coralSystem.m_hasCoralTrigger);

        m_elevator.setArmRadSupplier(m_arm.m_radSupplier);
        m_elevator.setHasCoralTrigger(m_coralSystem.m_hasCoralTrigger);
        m_toleranceTrigger = m_arm.m_toleranceTrigger.and(m_elevator.m_toleranceTrigger);
        m_currentState = State.DEFAULT;
        setStateCommand(State.DEFAULT).schedule();
    }

    public Command setStateCommand(State state) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> this.m_currentState = state),
                m_algaeSystem.setStateCommand(state.m_algaeVoltage),
                m_coralSystem.setStateCommand(state.m_coralVoltage),
                m_elevator.changeSetpointCommand(state.m_elevatorHeight),
                m_arm.changeSetpointCommand(state.m_armAngle))
                .andThen(new WaitUntilCommand(m_toleranceTrigger));
    }

    public Command intakeCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setStateCommand(State.INTAKE).until(m_coralSystem.m_hasCoralTrigger),
                        new WaitUntilCommand(m_coralSystem.m_hasCoralTrigger),
                        setStateCommand(State.DEFAULT)
                ),
                new PrintCommand(this.m_currentState.name() +
                        " is the current state of the robot.\n it cant intake"),
                () -> this.m_currentState.equals(State.DEFAULT)
        );
    }

    public Command scoreCoralCommand(int level, BooleanSupplier release) {
        State pose, score;
        switch (level) {
            case 1 -> {
                pose = State.PRE_L1;
                score = State.L1;
            }
            case 2 -> {
                pose = State.PRE_L2;
                score = State.L2;
            }
            case 3 -> {
                pose = State.PRE_L3;
                score = State.L3;
            }
            case 4 -> {
                pose = State.PRE_L4;
                score = State.L4;
            }
            default -> {
                return new PrintCommand("this is not a valid coral level: " + level);
            }
        }
        return new ConditionalCommand(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(m_coralSystem.m_hasCoralTrigger.negate()),
                        new SequentialCommandGroup(
                                setStateCommand(pose),
                                new WaitUntilCommand(release),
                                setStateCommand(score)
                        )
                ).andThen(setStateCommand(State.POST_L4), new WaitCommand(2), setStateCommand(State.DEFAULT)),
                new PrintCommand(this.m_currentState.name() +
                        " is the current state of the robot.\n it cant score"),
                () -> this.m_currentState.equals(State.DEFAULT));
    }

    public Command removeAlgaeCommand(int level, BooleanSupplier finish) {
        State state;
        switch (level) {
            case 2 -> state = State.ALGAE2;
            case 3 -> state = State.ALGAE3;
            default -> {
                return new PrintCommand("this is not a valid coral level: " + level);
            }
        }
        return new ConditionalCommand(
                new ParallelCommandGroup(
                        new WaitUntilCommand(finish),
                        setStateCommand(state)
                ).andThen(setStateCommand(State.DEFAULT)),
                new PrintCommand(this.m_currentState.name() +
                        " is the current state of the robot.\n it cant remove algae"),
                () -> this.m_currentState.equals(State.DEFAULT));
    }

    public Command coastCommand() {
        return new SequentialCommandGroup(
                m_arm.coastCommand(),
                m_elevator.coastCommand()
        );
    }

    @Log.NT
    public String currentState() {
        return this.m_currentState.name();
    }
}