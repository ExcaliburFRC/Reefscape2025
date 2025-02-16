package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.BooleanSupplier;

public class Superstructure implements Logged {
    private final Elevator m_elevator;
    private final Arm m_arm;
    private final Gripper m_gripper;
    public final Trigger toleranceTrigger;

    public Superstructure() {
        this.m_elevator = new Elevator();
        this.m_arm = new Arm();
        this.m_gripper = new Gripper();
        this.m_arm.setElevatorHeightSupplier(this.m_elevator.m_heightSupplier);

        this.m_elevator.setArmRadSupplier(this.m_arm.m_radSupplier);
        this.toleranceTrigger = m_arm.m_toleranceTrigger.and(m_elevator.m_toleranceTrigger);
    }

    public Command setStateCommand(State state, BooleanSupplier activateWheels) {
        return new SequentialCommandGroup(
                m_elevator.changeSetpointCommand(
                        state.m_elevatorHeight
                ),
                m_arm.changeSetpointCommand(
                        state.m_armAngle
                ),
                new WaitUntilCommand(
                        this.toleranceTrigger.and(activateWheels)
                ),
                m_gripper.manualCommand(
                        state.m_innerWheelsVoltage,
                        state.m_outWheelsVoltage
                )
        );
    }

    public Command intakeCommand(BooleanSupplier atPose) {
        return new SequentialCommandGroup(
                resetGripper(),
                setStateCommand(State.INTAKE, atPose).until(m_gripper.m_coralTrigger),
                setStateCommand(State.DEFAULT, () -> true)
        );
    }

    public Command scoreCoralCommand(int level, BooleanSupplier release) {
        State state;
        switch (level) {
            case 1 -> state = State.L1;
            case 2 -> state = State.L2;
            case 3 -> state = State.L3;
            case 4 -> state = State.L4;
            default -> {
                return new PrintCommand(level + " is not a valid coral level");
            }
        }
        return setStateCommand(state, release).until(
                m_gripper.m_coralTrigger.negate().debounce(2)).andThen(
                setStateCommand(State.DEFAULT, () -> true)
        );
    }

    public Command removeAlgaeCommand(int level, BooleanSupplier atPose) {
        if (level != 2 && level != 3) {
            return new PrintCommand(level + " is not a valid algae level :(");
        }
        State state = (level == 2) ? State.ALGAE2 : State.ALGAE3;
        return new SequentialCommandGroup(
                m_gripper.manualCommand(state.m_innerWheelsVoltage, state.m_outWheelsVoltage),
                setStateCommand(state, () -> true)
        ).until(atPose);
    }

    @Log.NT
    public boolean isSuperstructureAtSetpoint() {
        return this.toleranceTrigger.getAsBoolean();
    }

    public Command resetGripper() {
        return m_gripper.manualCommand(State.DEFAULT.m_innerWheelsVoltage, State.DEFAULT.m_outWheelsVoltage).withTimeout(0.05);
    }

    public Command toggleIdleMode() {
        return m_arm.coastCommand().alongWith(m_elevator.coastCommand());
    }

    public boolean hasCoral() {
        return m_gripper.hasCoral();
    }
}
