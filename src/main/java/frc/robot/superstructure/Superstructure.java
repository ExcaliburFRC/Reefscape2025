package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;

import java.util.function.BooleanSupplier;

public class Superstructure {
    private final Elevator m_elevator;
    private final Arm m_arm;
    private final Gripper m_gripper;
    public final Trigger toleranceTrigger;

    public Superstructure() {
        this.m_elevator = new Elevator();
        this.m_arm = new Arm();
        this.m_gripper = new Gripper();
        this.toleranceTrigger = m_arm.toleranceTrigger.and(m_elevator.toleranceTrigger);
    }

    public Command setStateCommand(State state, BooleanSupplier activateWheels) {
        Command command = new SequentialCommandGroup(
                m_elevator.setLengthCommand(state.m_elevatorHeight),
                m_arm.changeSetpointCommand(state.m_placerAngle),
                new WaitUntilCommand(this.toleranceTrigger.and(activateWheels)),
                m_gripper.manualCommand(state.m_innerWheelsVoltage, state.m_outWheelsVoltage)
        );
        return command;
    }
}
