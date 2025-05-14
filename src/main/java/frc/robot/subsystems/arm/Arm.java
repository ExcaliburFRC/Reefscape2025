package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.motor_types.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.robot.superstructure.State;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.*;
import static frc.excalib.control.motor.motor_specs.IdleState.*;
import static frc.robot.subsystems.arm.Constants.*;
import static frc.robot.superstructure.Constants.DEFAULT_ARM_ANGLE;
import static monologue.Annotations.Log;

public class Arm extends SubsystemBase implements Logged {

    // === Motor Setup ===
    private final MotorGroup m_motorGroup;
    private final CANcoder m_encoder;
    public final DoubleSupplier m_radSupplier;

    // === Arm Logic ===
    private final frc.excalib.mechanisms.Arm.Arm m_arm;
    private final ContinuousSoftLimit m_softLimit;
    private final InterpolatingDoubleTreeMap m_voltageLimitInterp;

    private DoubleSupplier m_elevatorHeightSupplier = () -> 0;
    private BooleanSupplier m_hasCoralTrigger = () -> true;
    private BooleanSupplier m_hasAlgaeTrigger;

    private double m_setpointAngle = DEFAULT_ARM_ANGLE;
    private boolean isAtTolerance = false;

    // === Triggers ===
    public final Trigger m_toleranceTrigger;
    public final Trigger m_defaultTrigger;

    // === Shuffleboard ===
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Superstructure");
    private final GenericEntry m_angleEntry = m_tab.add("Arm Angle", -1).getEntry();
    private final GenericEntry m_setpointEntry = m_tab.add("Arm Setpoint", -1).getEntry();
    private final GenericEntry m_atSetpointEntry = m_tab.add("Arm At Setpoint", false).getEntry();

    public Arm() {
        // Motors
        TalonFXMotor motor1 = new TalonFXMotor(FIRST_MOTOR_ID);
        TalonFXMotor motor2 = new TalonFXMotor(SECOND_MOTOR_ID);
        motor1.setInverted(REVERSE);
        motor2.setInverted(FORWARD);

        m_motorGroup = new MotorGroup(motor1, motor2);
        m_motorGroup.setVelocityConversionFactor(RPS_TO_RAD_PER_SEC);
        m_motorGroup.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);
        m_motorGroup.setIdleState(BRAKE);

        // Encoder
        m_encoder = new CANcoder(CAN_CODER_ID);
        m_radSupplier = () -> m_encoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD;
        m_motorGroup.setMotorPosition(m_radSupplier.getAsDouble());
        m_motorGroup.setCurrentLimit(0, 35);

        // Voltage profile for soft limit interpolation
        m_voltageLimitInterp = new InterpolatingDoubleTreeMap();
        m_voltageLimitInterp.put(0.0, CLOSED_MAX_VOL);
        m_voltageLimitInterp.put(0.2, 4.0);
        m_voltageLimitInterp.put(0.7, OPENED_MAX_VOL);

        // Arm logic
        m_arm = new frc.excalib.mechanisms.Arm.Arm(
            m_motorGroup,
            m_radSupplier,
            new SoftLimit(
                () -> -m_voltageLimitInterp.get(m_elevatorHeightSupplier.getAsDouble()),
                () -> m_voltageLimitInterp.get(m_elevatorHeightSupplier.getAsDouble())
            ),
            ANGLE_GAINS,
            new Mass(
                () -> Math.cos(m_radSupplier.getAsDouble()),
                () -> Math.sin(m_radSupplier.getAsDouble()),
                MASS
            )
        );

        // Dynamic soft limits based on elevator/conditions
        m_softLimit = new ContinuousSoftLimit(
            () -> {
                double height = m_elevatorHeightSupplier.getAsDouble();
                boolean coral = m_hasCoralTrigger.getAsBoolean();
                boolean algae = m_hasAlgaeTrigger != null && m_hasAlgaeTrigger.getAsBoolean();

                if (coral && height > ELEVATOR_LOWER_PROFILE_CORAL_TRIGGER && height < ELEVATOR_UPPER_PROFILE_CORAL_TRIGGER)
                    return MIN_PROFILE_CORAL_LIMIT;
                if (height > ELEVATOR_LOWER_PROFILE_TRIGGER && height < ELEVATOR_UPPER_PROFILE_TRIGGER)
                    return MIN_PROFILE_LIMIT;
                if (algae && height < ELEVATOR_ALGAE_SHIELD_TRIGGER)
                    return MIN_ALGAE_SHIELD_LIMIT;
                if (algae)
                    return MIN_ALGAE_SHAFT_LIMIT;
                return MIN_RAD_LIMIT;
            },
            () -> MAX_RAD_LIMIT
        );

        // Triggers
        m_toleranceTrigger = new Trigger(() -> this.isAtTolerance);
        m_defaultTrigger = new Trigger(() -> Math.abs(State.DEFAULT.m_armAngle - m_radSupplier.getAsDouble()) < TOLERANCE);

        setDefaultCommand(createDefaultCommand());
        SmartDashboard.putData(this);
    }

    // === Public Methods ===

    public Command manualCommand(DoubleSupplier output) {
        return m_arm.manualCommand(output, this);
    }

    public Command changeSetpointCommand(double setpoint) {
        return new RunCommand(() -> this.m_setpointAngle = setpoint)
            .until(() -> this.m_setpointAngle == setpoint);
    }

    public void setElevatorHeightSupplier(DoubleSupplier supplier) {
        m_elevatorHeightSupplier = supplier;
    }

    public void setHasCoralTrigger(BooleanSupplier trigger) {
        m_hasCoralTrigger = trigger;
    }

    public void setHasAlgaeTrigger(BooleanSupplier trigger) {
        m_hasAlgaeTrigger = trigger;
    }

    public Command coastCommand() {
        return new StartEndCommand(
            () -> m_motorGroup.setIdleState(COAST),
            () -> m_motorGroup.setIdleState(BRAKE)
        ).ignoringDisable(true).withName("Coast Command");
    }

    public Command sysIdCommand(boolean dynamic, SysIdRoutine.Direction direction, SysidConfig config) {
        return dynamic
            ? m_arm.sysIdDynamic(direction, this, m_arm.ANGLE_SUPPLIER, config, false)
            : m_arm.sysIdQuasistatic(direction, this, m_arm.ANGLE_SUPPLIER, config, false);
    }

    // === Logging ===

    @Log.NT public double getAngle() { return m_radSupplier.getAsDouble(); }
    @Log.NT public double getSetpoint() { return m_setpointAngle; }
    @Log.NT public double getLimitedSetpoint() { return m_softLimit.limit(m_setpointAngle); }
    @Log.NT public boolean atSetpoint() { return m_toleranceTrigger.getAsBoolean(); }

    public double getMinLimit() { return m_softLimit.getMinLimit(); }
    public double getMaxLimit() { return m_softLimit.getMaxLimit(); }

    @Override
    public void periodic() {
        m_angleEntry.setDouble(getAngle());
        m_setpointEntry.setDouble(getSetpoint());
        m_atSetpointEntry.setBoolean(atSetpoint());
    }

    private Command createDefaultCommand() {
        return m_arm.anglePositionControlCommand(
            () -> m_softLimit.limit(m_setpointAngle),
            at -> this.isAtTolerance = at,
            TOLERANCE,
            this
        ).withName("Default Arm Command");
    }
}
