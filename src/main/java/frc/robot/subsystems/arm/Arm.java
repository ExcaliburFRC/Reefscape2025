package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.robot.superstructure.State;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;
import static frc.excalib.control.motor.motor_specs.DirectionState.REVERSE;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;
import static frc.robot.subsystems.arm.Constants.*;
import static monologue.Annotations.Log;

public class Arm extends SubsystemBase implements Logged {
    private final TalonFXMotor m_firstMotor, m_secondMotor;
    private final MotorGroup m_motorGroup;
    private final frc.excalib.mechanisms.Arm.Arm m_arm;
    private final CANcoder m_encoder;
    public final DoubleSupplier m_radSupplier;
    private boolean isAtTolerance = false;
    public final Trigger m_toleranceTrigger;
    private double m_setpointAngle = State.DEFAULT.m_armAngle;
    private DoubleSupplier m_elevatorHeightSupplier;
    private ContinuousSoftLimit m_softLimit;

    private final ShuffleboardTab m_superstructureTab = Shuffleboard.getTab("Superstructure");
    private final GenericEntry m_armAngleEntry = m_superstructureTab.add("Arm Angle", -1).getEntry();
    private final GenericEntry m_armSetpointEntry = m_superstructureTab.add("Arm Setpoint", -1).getEntry();
    private final GenericEntry m_armAtSetpointEntry = m_superstructureTab.add("Arm At Setpoint", false).getEntry();

    public Arm() {
        m_firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);
        m_firstMotor.setInverted(REVERSE);
        m_secondMotor = new TalonFXMotor(SECOND_MOTOR_ID);
        m_secondMotor.setInverted(FORWARD);

        m_motorGroup = new MotorGroup(m_firstMotor, m_secondMotor);
        m_motorGroup.setVelocityConversionFactor(RPS_TO_RAD_PER_SEC);
        m_motorGroup.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        m_motorGroup.setIdleState(BRAKE);

        m_encoder = new CANcoder(CAN_CODER_ID);
        m_radSupplier = () -> m_encoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD;

        m_motorGroup.setMotorPosition(m_radSupplier.getAsDouble());
        m_motorGroup.setCurrentLimit(0, 25);

        m_arm = new frc.excalib.mechanisms.Arm.Arm(
                m_motorGroup,
                m_radSupplier,
                new SoftLimit(() -> -MAX_VOL_RAD_PER_SEC, () -> MAX_VOL_RAD_PER_SEC),
                ANGLE_GAINS,
                new Mass(() -> Math.cos(m_radSupplier.getAsDouble()), () -> Math.sin(m_radSupplier.getAsDouble()), MASS)
        );

        m_elevatorHeightSupplier = () -> 0;
        this.m_toleranceTrigger = new Trigger(() -> this.isAtTolerance);
        this.m_softLimit = new ContinuousSoftLimit(
                () -> m_elevatorHeightSupplier.getAsDouble() > ELEVATOR_HEIGHT_LIMIT_TRIGGER ?
                        EXTENDED_MIN_RAD_LIMIT :
                        CLOSED_MIN_RAD_LIMIT,
                () -> MAX_RAD_LIMIT
        );

        this.setDefaultCommand(defaultCommand());

        SmartDashboard.putData(this);
    }

    /**
     * @param output
     * @return a command that will run the arm with the given DoubleSupplier
     */
    public Command manualCommand(DoubleSupplier output) {
        return m_arm.manualCommand(output, this);
    }

    public Command changeSetpointCommand(double setpoint) {
        return new RunCommand(
                () -> {
                    this.m_setpointAngle = setpoint;
                }
        ).until(() -> this.m_setpointAngle == setpoint);
    }

    private Command defaultCommand() {
        return m_arm.anglePositionControlCommand(
                () -> m_softLimit.limit(this.m_setpointAngle),
                (isAtTolerance) -> {
                    this.isAtTolerance = isAtTolerance;
                },
                TOLERANCE,
                this
        ).withName("Default Command");
    }

    public void setElevatorHeightSupplier(DoubleSupplier elevatorHeightSupplier) {
        this.m_elevatorHeightSupplier = elevatorHeightSupplier;
    }

    public Command coastCommand() {
        return new StartEndCommand(
                () -> m_motorGroup.setIdleState(COAST),
                () -> m_motorGroup.setIdleState(BRAKE)
        ).ignoringDisable(true).withName("Coast Command");
    }

    @Log.NT
    public double getAngle() {
        return m_radSupplier.getAsDouble();
    }

    @Log.NT
    public boolean atSetpoint() {
        return m_toleranceTrigger.getAsBoolean();
    }

    @Log.NT
    public double getSetpoint() {
        return m_setpointAngle;
    }

    @Log.NT
    public double getLimitedSetpoint() {
        return m_softLimit.limit(getSetpoint());
    }

    @Override
    public void periodic() {
        m_armAngleEntry.setDouble(getAngle());
        m_armSetpointEntry.setDouble(getSetpoint());
        m_armAtSetpointEntry.setBoolean(atSetpoint());
    }

    public Command sysIdCommand(boolean dynamic, SysIdRoutine.Direction direction, SysidConfig sysidConfig) {
        if (dynamic) return m_arm.sysIdDynamic(direction, this, m_arm.ANGLE_SUPPLIER, sysidConfig, false);
        return m_arm.sysIdQuasistatic(direction, this, m_arm.ANGLE_SUPPLIER, sysidConfig, false);
    }
}

