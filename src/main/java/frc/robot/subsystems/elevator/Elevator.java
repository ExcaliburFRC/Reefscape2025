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
import static frc.robot.subsystems.elevator.Constants.*;
import static monologue.Annotations.Log;

public class Elevator extends SubsystemBase implements Logged {
    private final TalonFXMotor m_firstMotor, m_secondMotor;
    private final MotorGroup m_motorGroup;
    private final LinearExtension m_extensionMechanism;
    private double m_setpoint;
    public final Trigger m_toleranceTrigger;
    public final Trigger defultTrigger;
    public DoubleSupplier m_heightSupplier;
    private DoubleSupplier m_armRadSupplier;
    private SoftLimit m_softLimit;
    private double m_prevVel = 0;
    private double m_accel = 0;
    private final Trigger m_closedTrigger;

    private final ShuffleboardTab m_superstructureTab = Shuffleboard.getTab("Superstructure");
    private final GenericEntry m_elevatorHeightEntry = m_superstructureTab.add("Elevator Height", -1).getEntry();
    private final GenericEntry m_elevatorSetpointEntry = m_superstructureTab.add("Elevator Setpoint", -1).getEntry();
    private final GenericEntry m_elevatorAtSetpointEntry = m_superstructureTab.add("Elevator At Setpoint", false).getEntry();

    public Elevator() {
        m_firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);
        m_firstMotor.setInverted(REVERSE);
        m_secondMotor = new TalonFXMotor(SECOND_MOTOR_ID);
        m_secondMotor.setInverted(FORWARD);

        m_motorGroup = new MotorGroup(m_firstMotor, m_secondMotor);
        m_motorGroup.setPositionConversionFactor(ROTATIONS_TO_METERS);
        m_motorGroup.setVelocityConversionFactor(ROTATIONS_TO_METERS);
        m_motorGroup.setCurrentLimit(0, 50);
        m_motorGroup.setIdleState(IdleState.BRAKE);
        m_setpoint = 0;
        this.m_closedTrigger = new Trigger(() -> getCurrent() > STALL_THRESHOLD && m_setpoint == MIN_HEIGHT && Math.abs(getHeight()) <= 0.1).debounce(0.35);
        this.m_closedTrigger.onTrue(new InstantCommand(() -> m_motorGroup.setMotorPosition(0)).andThen(new PrintCommand("reset elevator")));

        m_extensionMechanism = new LinearExtension(
                m_motorGroup,
                this.m_motorGroup::getMotorPosition,
                () -> ELEVATOR_ANGLE,
                ELEVATOR_GAINS,
                CONSTRAINTS,
                TOLERANCE
        );

        m_heightSupplier = m_extensionMechanism::logPosition;

        m_toleranceTrigger = new Trigger(() -> Math.abs(this.m_setpoint - m_heightSupplier.getAsDouble()) < TOLERANCE);
        defultTrigger = new Trigger(() -> Math.abs(State.DEFAULT.m_elevatorHeight - m_heightSupplier.getAsDouble()) < TOLERANCE);

        this.m_armRadSupplier = () -> 0;

        this.m_softLimit = new SoftLimit(
                () -> {
                    if (m_armRadSupplier.getAsDouble() > ARM_COLLISION_RAD) {
                        return MIN_HEIGHT;
                    }
                    return m_heightSupplier.getAsDouble() > UPPER_MIN_HEIGHT ? UPPER_MIN_HEIGHT : MIN_HEIGHT;
                },
                () -> {
                    if (m_armRadSupplier.getAsDouble() > ARM_COLLISION_RAD) {
                        return MAX_HEIGHT;
                    }
                    return m_heightSupplier.getAsDouble() <= LOWER_MAX_HEIGHT ? LOWER_MAX_HEIGHT : MAX_HEIGHT;
                }
        );

        this.setDefaultCommand(defaultCommand());

        SmartDashboard.putData(this);
    }

    public Command manualCommand(DoubleSupplier output) {
        return m_extensionMechanism.manualCommand(output, this);
    }

    private Command defaultCommand() {
        return m_extensionMechanism.extendCommand(
                () -> {
                    double setpoint = m_softLimit.limit(this.m_setpoint);
                    return setpoint;
                },
                this
        ).withName("Default Elevator Command");
    }

    public Command changeSetpointCommand(double length) {
        return new RunCommand(
                () -> this.m_setpoint = length
        ).until(() -> m_setpoint == length);
    }

    public Command coastCommand() {
        return new StartEndCommand(
                () -> m_motorGroup.setIdleState(IdleState.COAST),
                () -> m_motorGroup.setIdleState(IdleState.BRAKE)
        ).ignoringDisable(true).withName("Coast Command");
    }

    public Command resetHeightCommand() {
        return new InstantCommand(() -> {
            m_motorGroup.setMotorPosition(0);
            System.out.println("reset elevator height");
        }).ignoringDisable(true);
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

    public void setArmRadSupplier(DoubleSupplier armRadSupplier) {
        this.m_armRadSupplier = armRadSupplier;
    }

    public Command sysIdCommand(boolean dynamic, SysIdRoutine.Direction direction, SysidConfig sysidConfig) {
        if (dynamic) return m_extensionMechanism.sysIdDynamic(direction, this, m_heightSupplier, sysidConfig, true);
        return m_extensionMechanism.sysIdQuasistatic(direction, this, m_heightSupplier, sysidConfig, true);
    }

    @Log.NT
    public double getLimitedSetpoint() {
        return m_softLimit.limit(getSetpoint());
    }
    @Log.NT
    public double getMaxLimit() {
        return m_softLimit.getMaxLimit();
    }
    @Log.NT
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
}