package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.linear_extension.LinearExtension;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.Constants.*;

public class Elevator extends SubsystemBase {
    private final TalonFXMotor m_motor;
    private final LinearExtension m_extensionMechanism;

    public Elevator(){
        m_motor = new TalonFXMotor(MOTOR_ID);
        m_extensionMechanism = new LinearExtension(
                m_motor,
                m_motor::getMotorPosition,
                ()-> ELEVATOR_ANGLE,
                ELEVATOR_GAINS,
                UPWARD_CONSTRAINTS,
                DOWNWARD_CONSTRAINTS
        );

        this.setDefaultCommand(setLengthCommand(MIN_LENGTH));
    }

    public Command manualCommand(DoubleSupplier output){
        return m_extensionMechanism.manualCommand(output, this);
    }
    public Command setLengthCommand(double length){
        return m_extensionMechanism.extendCommand(()->length, this);
    }
}
