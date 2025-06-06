package frc.robot.subsystems.elevator;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.linear_extension.LinearExtension;
import frc.robot.superstructure.State;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;
import static frc.excalib.control.motor.motor_specs.DirectionState.REVERSE;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.robot.subsystems.elevator.Constants.*;
import static monologue.Annotations.Log;

public class Elevator extends SubsystemBase implements Logged {

    private final MotorGroup m_motorGroup;
    private final LinearExtension m_extensionMechanism;
    private final SoftLimit m_softLimit;
    private final ShuffleboardTab m_superstructureTab = Shuffleboard.getTab("Superstructure");

    private final GenericEntry m_elevatorHeightEntry =
        m_superstructureTab.add("Elevator Height", -1).getEntry();
    private final GenericEntry m_elevatorSetpointEntry =
        m_superstructureTab.add("Elevator Setpoint", -1).getEntry();
    private final GenericEntry m_elevatorAtSetpointEntry =
        m_superstructureTab.add("Elevator At Setpoint", false).getEntry();

    private double m_setpoint;
    private double m_prevVel = 0;
    private double m_accel = 0;

    public final Trigger m_toleranceTrigger, m_defaultTrigger, m_closedTrigger;
    public DoubleSupplier m_heightSupplier, m_armRadSupplier;
    private Trigger m_hasCoralTrigger = new Trigger(() -> true);

    public Elevator() {
        TalonFXMotor m_firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);
        m_firstMotor.setInverted(REVERSE);

        TalonFXMotor m_secondMotor = new TalonFXMotor(SECOND_MOTOR_ID);
        m_secondMotor.setInverted(FORWARD);

        m_motorGroup = new MotorGroup(m_firstMotor, m_secondMotor);
        m_motorGroup.setPositionConversionFactor(ROTATIONS_TO_METERS);
        m_motorGroup.setVelocityConversionFactor(ROTATIONS_TO_METERS);
        m_motorGroup.setCurrentLimit(0, 50);
        m_motorGroup.setIdleState(BRAKE);

        m_setpoint = 0;

        m_closedTrigger = new Trigger(() ->
            getCurrent() > STALL_THRESHOLD &&
            m_setpoint == MIN_HEIGHT_LIMIT &&
            Math.abs(getHeight()) <= 0.1
        ).debounce(0.35);
        m_closedTrigger.onTrue(resetHeightCommand());

        m_heightSupplier = m_firstMotor::getMotorPosition;

        m_extensionMechanism = new LinearExtension(
            m_motorGroup,
            m_heightSupplier,
            () -> ELEVATOR_ANGLE,
            ELEVATOR_GAINS,
            CONSTRAINTS,
            TOLERANCE
        );

        m_toleranceTrigger = new Trigger(() ->
            Math.abs(m_setpoint - m_heightSupplier.getAsDouble()) < TOLERANCE
        );

        m_defaultTrigger = new Trigger(() ->
            Math.abs(State.DEFAULT.m_elevatorHeight - m_heightSupplier.getAsDouble()) < TOLERANCE
        );

        m_armRadSupplier = () -> 0;

        m_softLimit = new SoftLimit(
            () -> {
                double collisionAngle = m_hasCoralTrigger.getAsBoolean() ?
                    CORAL_COLLISION_TRIGGER : ARM_COLLISION_TRIGGER;

                boolean inCollision = m_armRadSupplier.getAsDouble() < collisionAngle;
                if (!inCollision) return MIN_HEIGHT_LIMIT;

                return m_heightSupplier.getAsDouble() > UPPER_MIN_LIMIT ?
                    UPPER_MIN_LIMIT : MIN_HEIGHT_LIMIT;
            },
            () -> {
                double collisionAngle = m_hasCoralTrigger.getAsBoolean() ?
                    CORAL_COLLISION_TRIGGER : ARM_COLLISION_TRIGGER;

                boolean inCollision = m_armRadSupplier.getAsDouble() < collisionAngle;
                if (!inCollision) return MAX_HEIGHT_LIMIT;

                return m_heightSupplier.getAsDouble() < LOWER_MAX_LIMIT ?
                    LOWER_MAX_LIMIT : MAX_HEIGHT_LIMIT;
            }
        );

        setDefaultCommand(defaultCommand());

        SmartDashboard.putData(this);
    }

    private Command defaultCommand() {
        return m_extensionMechanism.extendCommand(
            () -> m_softLimit.limit(m_setpoint),
            this
        ).withName("Default Elevator Command");
    }

    public Command manualCommand(DoubleSupplier output) {
        return m_extensionMechanism.manualCommand(output, this);
    }

    public Command changeSetpointCommand(double length) {
        return new RunCommand(() -> m_setpoint = length)
            .until(() -> m_setpoint == length);
    }

    public Command coastCommand() {
        return new StartEndCommand(
            () -> m_motorGroup.setIdleState(IdleState.COAST),
            () -> m_motorGroup.setIdleState(BRAKE)
        ).ignoringDisable(true).withName("Coast Command");
    }

    public Command resetHeightCommand() {
        return new InstantCommand(() -> {
            m_motorGroup.setMotorPosition(0);
            System.out.println("reset elevator height");
        }).ignoringDisable(true);
    }

    public Command sysIdCommand(boolean dynamic, SysIdRoutine.Direction direction, SysidConfig sysidConfig) {
        return dynamic ?
            m_extensionMechanism.sysIdDynamic(direction, this, m_heightSupplier, sysidConfig, true) :
            m_extensionMechanism.sysIdQuasistatic(direction, this, m_heightSupplier, sysidConfig, true);
    }

    public void setArmRadSupplier(DoubleSupplier armRadSupplier) {
        this.m_armRadSupplier = armRadSupplier;
    }

    public void setHasCoralTrigger(Trigger hasCoralTrigger) {
        this.m_hasCoralTrigger = hasCoralTrigger;
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

    @Log.NT
    public double getCurrent() {
        return m_motorGroup.getCurrent();
    }

    public double getLimitedSetpoint() {
        return m_softLimit.limit(getSetpoint());
    }

    public double getMaxLimit() {
        return m_softLimit.getMaxLimit();
    }

    public double getMinLimit() {
        return m_softLimit.getMinLimit();
    }

    @Override
    public void periodic() {
        m_accel = (m_extensionMechanism.logVelocity() - m_prevVel) / 0.02;
        m_prevVel = m_extensionMechanism.logVelocity();

        m_elevatorHeightEntry.setDouble(getHeight());
        m_elevatorSetpointEntry.setDouble(getSetpoint());
        m_elevatorAtSetpointEntry.setBoolean(atSetpoint());
    }

    @Log.NT
    public boolean collisionAngle() {
        return m_hasCoralTrigger.getAsBoolean();
    }
}
