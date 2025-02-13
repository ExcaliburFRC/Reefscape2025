package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.linear_extension.LinearExtension;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.REVERSE;
import static frc.robot.subsystems.elevator.Constants.*;
import static monologue.Annotations.Log;

public class Elevator extends SubsystemBase implements Logged {
    private final TalonFXMotor m_firstMotor, m_secondMotor;
    private final MotorGroup m_motorGroup;
    private final LinearExtension m_extensionMechanism;
    private double m_setpoint;
    public final Trigger m_toleranceTrigger;
    public DoubleSupplier m_heightSupplier;
    private DoubleSupplier armRadSupplier;
    private SoftLimit m_softLimit;
    private double m_prevVel = 0;
    private double m_accel = 0;

    public Elevator() {
        m_firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);
        m_firstMotor.setInverted(REVERSE);
        m_secondMotor = new TalonFXMotor(SECOND_MOTOR_ID);

        m_motorGroup = new MotorGroup(m_firstMotor, m_secondMotor);
        m_motorGroup.setPositionConversionFactor(ROTATIONS_TO_METERS);
        m_motorGroup.setVelocityConversionFactor(ROTATIONS_TO_METERS);

        m_setpoint = 0.003;

        m_extensionMechanism = new LinearExtension(
                m_motorGroup,
                this.m_motorGroup::getMotorPosition,
                () -> ELEVATOR_ANGLE,
                ELEVATOR_GAINS,
                CONSTRAINTS
        );
        m_heightSupplier = m_extensionMechanism::logPosition;
        m_toleranceTrigger = new Trigger(
                () -> Math.abs(this.m_setpoint - m_extensionMechanism.logPosition()) < TOLERANCE
        );
        this.armRadSupplier = () -> 0;
        this.m_softLimit = new SoftLimit(() -> {
            if (armRadSupplier.getAsDouble() > ARM_COLLISION_RAD) return MIN_HEIGHT;
            return m_heightSupplier.getAsDouble() > UPPER_MIN_HEIGHT? UPPER_MIN_HEIGHT : MIN_HEIGHT;
        }, () -> {
            if (armRadSupplier.getAsDouble() > ARM_COLLISION_RAD) return MAX_HEIGHT;
            return m_heightSupplier.getAsDouble() < LOWER_MAX_HEIGHT? LOWER_MAX_HEIGHT : MAX_HEIGHT;
        });
        this.setDefaultCommand(defaultCommand());
    }

    public Command manualCommand(DoubleSupplier output) {
        return m_extensionMechanism.manualCommand(output, this);
    }

    private Command defaultCommand() {
        return m_extensionMechanism.extendCommand(
                () -> this.m_setpoint,//m_softLimit.limit(this.m_setpoint),
                this
        );
    }

    public Command changeSetpointCommand(double length) {
        return new InstantCommand(
                () -> this.m_setpoint = length, this
        );
    }

    @Log.NT
    public double getHeight() {
        return m_heightSupplier.getAsDouble();
    }

    @Log.NT
    public double getSetpoint() {
        return m_setpoint;
    }

    @Log.NT
    public boolean atSetpoint() {
        return m_toleranceTrigger.getAsBoolean();
    }

    @Log.NT
    public double getAccel() {
        return m_accel;
    }

    public void setArmRadSupplier(DoubleSupplier armRadSupplier) {
        this.armRadSupplier = armRadSupplier;
    }

    public Command sysIdCommand(boolean dynamic, SysIdRoutine.Direction direction, SysidConfig sysidConfig) {
        if (dynamic) return m_extensionMechanism.sysIdDynamic(direction, this, m_heightSupplier, sysidConfig, true);
        return m_extensionMechanism.sysIdQuasistatic(direction, this, m_heightSupplier, sysidConfig, true);
    }

    @Override
    public void periodic() {
        m_accel = (m_extensionMechanism.logVelocity() - m_prevVel) / 0.02;
        m_prevVel = m_extensionMechanism.logVelocity();
    }
}