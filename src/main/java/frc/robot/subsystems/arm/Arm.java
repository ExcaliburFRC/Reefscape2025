package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.FORWARD;
import static frc.excalib.control.motor.motor_specs.DirectionState.REVERSE;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;
import static frc.robot.subsystems.arm.Constants.*;
import static frc.robot.superstructure.Constants.DEFAULT_ARM_ANGLE;
import static monologue.Annotations.Log;

public class Arm extends SubsystemBase implements Logged {
    private final TalonFXMotor m_firstMotor, m_secondMotor;
    private final MotorGroup m_motorGroup;
    private final frc.excalib.mechanisms.Arm.Arm m_arm;
    private final CANcoder m_encoder;
    public final DoubleSupplier m_radSupplier;
    private boolean isAtTolerance = false;
    public final Trigger m_toleranceTrigger;
    public final Trigger m_defultTrigger;
    private double m_setpointAngle = DEFAULT_ARM_ANGLE;
    private DoubleSupplier m_elevatorHeightSupplier;
    private BooleanSupplier m_hasCoralTrigger, m_hasAlgaeTrigger;
    private ContinuousSoftLimit m_softLimit;
    private InterpolatingDoubleTreeMap m_voltageLimitInterpulator;
    private final ShuffleboardTab m_superstructureTab = Shuffleboard.getTab("Superstructure");
    private final GenericEntry m_armAngleEntry = m_superstructureTab.add("Arm Angle", -1).getEntry();
    private final GenericEntry m_armSetpointEntry = m_superstructureTab.add("Arm Setpoint", -1).getEntry();
    private final GenericEntry m_armAtSetpointEntry = m_superstructureTab.add("Arm At Setpoint", false).getEntry();

    public Arm() {
        m_hasCoralTrigger = () -> true;
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
        m_motorGroup.setCurrentLimit(0, 35);

        m_voltageLimitInterpulator = new InterpolatingDoubleTreeMap();
        m_voltageLimitInterpulator.put(0.0, CLOSED_MAX_VOL);
        m_voltageLimitInterpulator.put(0.2, 4.0);
        m_voltageLimitInterpulator.put(0.7, OPENED_MAX_VOL);
        m_arm = new frc.excalib.mechanisms.Arm.Arm(
                m_motorGroup,
                m_radSupplier,
                new SoftLimit(
                        () -> -m_voltageLimitInterpulator.get(m_elevatorHeightSupplier.getAsDouble()),
                        () -> m_voltageLimitInterpulator.get(m_elevatorHeightSupplier.getAsDouble())),
                ANGLE_GAINS,
                new Mass(() -> Math.cos(m_radSupplier.getAsDouble()), () -> Math.sin(m_radSupplier.getAsDouble()), MASS)
        );

        m_elevatorHeightSupplier = () -> 0;

        this.m_toleranceTrigger = new Trigger(() -> this.isAtTolerance);
        m_defultTrigger = new Trigger(() -> Math.abs(State.DEFAULT.m_armAngle - m_radSupplier.getAsDouble()) < TOLERANCE);

        this.m_softLimit = new ContinuousSoftLimit(
                () -> {
                    double elevatorHeight = m_elevatorHeightSupplier.getAsDouble();
                    boolean hasCoral = m_hasCoralTrigger.getAsBoolean();
                    boolean hasAlgae = m_hasAlgaeTrigger.getAsBoolean();
                    if (hasCoral &&
                            elevatorHeight > ELEVATOR_LOWER_PROFILE_CORAL_TRIGGER &&
                            elevatorHeight < ELEVATOR_UPPER_PROFILE_CORAL_TRIGGER) {
                        return MIN_PROFILE_CORAL_LIMIT;
                    }
                    if (elevatorHeight > ELEVATOR_LOWER_PROFILE_TRIGGER &&
                            elevatorHeight < ELEVATOR_UPPER_PROFILE_TRIGGER) {
                        return MIN_PROFILE_LIMIT;
                    }
                    if (hasAlgae && elevatorHeight < ELEVATOR_ALGAE_SHIELD_TRIGGER) {
                        return MIN_ALGAE_SHIELD_LIMIT;
                    }
                    if (hasAlgae) {
                        return MIN_ALGAE_SHAFT_LIMIT;
                    }
                    return MIN_RAD_LIMIT;
                },
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
                () -> this.m_setpointAngle = setpoint
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
        ).withName("Default Arm Command");
    }

    public void setElevatorHeightSupplier(DoubleSupplier elevatorHeightSupplier) {
        this.m_elevatorHeightSupplier = elevatorHeightSupplier;
    }

    public void setHasCoralTrigger(BooleanSupplier hasCoralTrigger) {
        m_hasCoralTrigger = hasCoralTrigger;
    }

    public void setHasAlgaeTrigger(BooleanSupplier hasAlgaeTrigger) {
        m_hasAlgaeTrigger = hasAlgaeTrigger;
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
        m_armAngleEntry.setDouble(getAngle());
        m_armSetpointEntry.setDouble(getSetpoint());
        m_armAtSetpointEntry.setBoolean(atSetpoint());
    }

    public Command sysIdCommand(boolean dynamic, SysIdRoutine.Direction direction, SysidConfig sysidConfig) {
        if (dynamic) return m_arm.sysIdDynamic(direction, this, m_arm.ANGLE_SUPPLIER, sysidConfig, false);
        return m_arm.sysIdQuasistatic(direction, this, m_arm.ANGLE_SUPPLIER, sysidConfig, false);
    }
}

