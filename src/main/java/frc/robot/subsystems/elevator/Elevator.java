package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.linear_extension.LinearExtension;

import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.REVERSE;
import static frc.robot.subsystems.elevator.Constants.*;

public class Elevator extends SubsystemBase {
    private final TalonFXMotor m_firstMotor, m_secondMotor;
    MotorGroup m_motorGroup;
    private final LinearExtension m_extensionMechanism;

    public Elevator() {
        m_firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);
        m_secondMotor = new TalonFXMotor(SECOND_MOTOR_ID);
        m_secondMotor.setInverted(REVERSE);
        m_motorGroup = new MotorGroup(m_firstMotor, m_secondMotor);


        m_extensionMechanism = new LinearExtension(
                m_motorGroup,
                m_motorGroup::getMotorPosition,
                () -> ELEVATOR_ANGLE,
                ELEVATOR_GAINS,
                UPWARD_CONSTRAINTS,
                DOWNWARD_CONSTRAINTS
        );

        this.setDefaultCommand(setLengthCommand(MIN_LENGTH));
    }

    public Command manualCommand(DoubleSupplier output) {
        return m_extensionMechanism.manualCommand(output, this);
    }

    public Command setLengthCommand(double length) {
        return m_extensionMechanism.extendCommand(() -> length, this);
    }
}
