package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algae.AlgaeSystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.coral.CoralSystem;
import frc.robot.subsystems.elevator.Elevator;
import monologue.Annotations.Log;
import monologue.Logged;

public class Superstructure implements Logged{
    private Elevator m_elevator;
    private Arm m_arm;
    private CoralSystem m_coralSystem;
    private AlgaeSystem m_algaeSystem;
    public Trigger m_toleranceTrigger;
    private State m_currentState;
    private Command runningCommand = null;
    private Command runningStateCommand = null;

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
        scheduleExclusiveStateCommand(State.DEFAULT).schedule();
    }

    private State getCoralPreState(int level) {
        State state;
        switch (level) {
            case 1 -> state = State.PRE_L1;
            case 2 -> state = State.PRE_L2;
            case 3 -> state = State.PRE_L3;
            case 4 -> state = State.PRE_L4;
            default -> state = State.DEFAULT;
        }
        return state;
    }

    private State getIntakeAlgaeState(int level) {
        State state;
        switch (level) {
            case 2 -> state = State.INTAKE_ALGAE2;
            case 3 -> state = State.INTAKE_ALGAE3;
            default -> state = State.DEFAULT;
        }
        return state;
    }

    private State getPreScoreAlgaeState(int level) {
        State state;
        switch (level) {
            case 1 -> state = State.PRE_PROCESSOR;
            case 4 -> state = State.PRE_NET;
            default -> state = State.DEFAULT;
        }
        return state;
    }


    private Command setStateCommand(State state) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> this.m_currentState = state),
                m_algaeSystem.setStateCommand(state.m_algaeVoltage),
                m_coralSystem.setStateCommand(state.m_coralVoltage),
                m_elevator.changeSetpointCommand(state.m_elevatorHeight),
                m_arm.changeSetpointCommand(state.m_armAngle))
                .andThen(new WaitUntilCommand(m_toleranceTrigger)).withName("");
    }

    private Command scheduleExclusiveCommand(Command newCommand) {
        return new FunctionalCommand(
                () -> {
                    if (runningCommand != null && runningCommand.isScheduled()) {
                        runningCommand.cancel();
                    }
                    runningCommand = newCommand;
                    runningCommand.schedule();
                },
                () -> {
                }, // No execute action needed
                (interrupted) -> {
                }, // No special end behavior needed
                () -> !runningCommand.isScheduled() // Ends when the command is no longer scheduled
        );
    }

    private Command scheduleExclusiveStateCommand(Command stateCommand) {
        return new FunctionalCommand(
                () -> {
                    if (runningStateCommand != null && runningStateCommand.isScheduled()) {
                        runningStateCommand.cancel();
                    }
                    runningStateCommand = stateCommand;
                    runningStateCommand.schedule();
                },
                () -> {}, // No execute action needed
                (interrupted) -> {}, // No special end behavior needed
                () -> !runningStateCommand.isScheduled() // Ends when the command is no longer scheduled
        );
    }

    private Command scheduleExclusiveStateCommand(State state) {
        return scheduleExclusiveStateCommand(setStateCommand(state));
    }

    private Command scoreCoralCommand(State score, State after) {
        return
                new SequentialCommandGroup(
                        scheduleExclusiveStateCommand(score),
                        new WaitUntilCommand(m_coralSystem.m_hasCoralTrigger.negate()).withTimeout(0.1),
                        scheduleExclusiveStateCommand(after),
                        new WaitUntilCommand(m_coralSystem.m_hasCoralTrigger.negate().debounce(0.2))
                );
    }

    public Command alignToCoralCommand(int level) {
        State state = getCoralPreState(level);
        if (state.equals(State.DEFAULT))
            return new PrintCommand(level + " is not a valid coral level, cant align to it");
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        scheduleExclusiveStateCommand(state),
                        new PrintCommand("cant go to level " + level + " no coral exist"),
                        this.m_coralSystem.m_hasCoralTrigger
                ));
    }

    public Command scoreCoralCommand(int level) {
        State score, after;
        switch (level) {
            case 1 -> {
                score = State.L1;
                after = State.POST_L1;
            }
            case 2 -> {
                score = State.L2;
                after = State.POST_L2;
            }
            case 3 -> {
                score = State.L3;
                after = State.POST_L3;
            }
            case 4 -> {
                score = State.L4;
                after = State.POST_L4;
            }
            default -> {
                return new PrintCommand(level + " is not a valid coral level, cant score to it");
            }
        }
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new WaitUntilCommand(m_toleranceTrigger).andThen(scoreCoralCommand(score, after)),
                        new PrintCommand("has no coral or not at correct state"),
                        m_coralSystem.m_hasCoralTrigger.and(() -> getCoralPreState(level).equals(m_currentState))
                )
        );
    }

    public Command intakeCoralCommand() {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                scheduleExclusiveStateCommand(State.INTAKE).until(m_coralSystem.m_hasCoralTrigger),
                                new WaitUntilCommand(m_coralSystem.m_hasCoralTrigger)
                        ),
                        new PrintCommand("cant intake"),
                        m_coralSystem.m_hasCoralTrigger.negate().and(() -> !m_algaeSystem.hasAlgae())));
    }

    public Command intakeAlgaeCommand(int level) {
        State state = getIntakeAlgaeState(level);
        if (state.equals(State.DEFAULT))
            return new PrintCommand(level + " is not a valid algae level, cant align to it");
        return scheduleExclusiveCommand(scheduleExclusiveStateCommand(state));
    }

    public Command alignToAlgaeCommand(int level) {
        State state = getPreScoreAlgaeState(level);
        if (state.equals(State.DEFAULT))
            return new PrintCommand(level + " is not a valid coral level, cant align to it");
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        scheduleExclusiveStateCommand(state),
                        new PrintCommand("cant go to level " + level + " no coral exist"),
                        this.m_algaeSystem.m_hasAlgaeTrigger
                ));
    }

    private Command scoreAlgaeCommand(State score, State after) {
        return scheduleExclusiveCommand(
                new SequentialCommandGroup(
                        scheduleExclusiveStateCommand(score),
                        new WaitUntilCommand(m_algaeSystem.m_hasAlgaeTrigger.negate().debounce(0.5)),
                        scheduleExclusiveCommand(scheduleExclusiveStateCommand(after))
                )
        );
    }

    public Command scoreAlgaeCommand(int level) {
        State score, after;
        switch (level) {
            case 1 -> {
                score = State.PROCESSOR;
                after = State.POST_PROCESSOR;
            }
            case 4 -> {
                score = State.NET;
                after = State.POST_NET;
            }
            default -> {
                return new PrintCommand(level + " is not a valid coral level, cant score to it");
            }
        }
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        new WaitUntilCommand(m_toleranceTrigger).andThen(scoreAlgaeCommand(score, after)),
                        new PrintCommand("has no coral or not at correct state"),
                        m_algaeSystem.m_hasAlgaeTrigger.and(() -> getPreScoreAlgaeState(level).equals(m_currentState))
                ));
    }

    public Command collapseCommand() {
        return scheduleExclusiveCommand(
                new ConditionalCommand(
                        scheduleExclusiveStateCommand(State.ALGAE_DEFAULT),
                        scheduleExclusiveStateCommand(State.DEFAULT),
                        () -> m_algaeSystem.hasAlgae()
                ));
    }

    public Command coastCommand() {
        return scheduleExclusiveCommand(
                new ParallelCommandGroup(
                        m_arm.coastCommand(),
                        m_elevator.coastCommand()
                )
        );
    }

    public Command startAutomationCommand() {
        return scheduleExclusiveCommand(scheduleExclusiveStateCommand(State.AUTOMATION_DEFAULT));
    }

    @Log.NT
    public String currentState() {
        return this.m_currentState.name();
    }

    public Trigger hasAlgaeTrigger() {
        return m_algaeSystem.m_hasAlgaeTrigger;
    }

    public Trigger hasCoralTrigger() {
        return m_coralSystem.m_hasCoralTrigger;
    }
    public Command toggleCoralCommand(){
        return m_coralSystem.toggleCoralCommand();
    }
}