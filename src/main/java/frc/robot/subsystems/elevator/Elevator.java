package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.linear_extension.LinearExtension;

import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.REVERSE;
import static frc.robot.subsystems.elevator.Constants.*;

public class Elevator extends SubsystemBase {
    private final TalonFXMotor m_firstMotor, m_secondMotor;
    private final MotorGroup m_motorGroup;
    private final LinearExtension m_extensionMechanism;
    private double lengthSetpoint;
    public final Trigger toleranceTrigger;

    public Elevator() {
        m_firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);
        m_secondMotor = new TalonFXMotor(SECOND_MOTOR_ID);
        m_secondMotor.setInverted(REVERSE);
        m_motorGroup = new MotorGroup(m_firstMotor, m_secondMotor);

        m_extensionMechanism = new LinearExtension(
                m_motorGroup,
                this.m_motorGroup::getMotorPosition,
                () -> ELEVATOR_ANGLE,
                ELEVATOR_GAINS,
                UPWARD_CONSTRAINTS,
                DOWNWARD_CONSTRAINTS
        );

        toleranceTrigger = new Trigger(
                () -> Math.abs(this.lengthSetpoint - m_extensionMechanism.logPosition()) < TOLERANCE
        );
        this.setDefaultCommand(goToDefaultLengthCommand());
    }

    public Command manualCommand(DoubleSupplier output) {
        return m_extensionMechanism.manualCommand(output, this);
    }

    public Command goToDefaultLengthCommand() {
        return m_extensionMechanism.extendCommand(
                this::getSetpoint,
                this
        );
    }

    public Command setLengthCommand(double length) {
        return new InstantCommand(
                () -> this.lengthSetpoint = length, this);
    }

    private double getSetpoint() {
        return lengthSetpoint;
    }
}
