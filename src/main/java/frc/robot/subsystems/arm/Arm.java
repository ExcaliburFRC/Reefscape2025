package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.arm.Constants.*;

public class Arm extends SubsystemBase {
    private final TalonFXMotor m_firstMotor, m_secondMotor;
    private final MotorGroup m_motorGroup;
    private final frc.excalib.mechanisms.Arm.Arm m_arm;
    private final CANcoder m_encoder;
    public final DoubleSupplier m_radSupplier;
    private boolean isAtTolerance = false;
    public final Trigger m_toleranceTrigger;
    private double setpointAngle = 0;
    private DoubleSupplier elevatorHeightSupplier;
    private ContinuousSoftLimit m_softLimit;

    public Arm() {
        m_firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);
        m_secondMotor = new TalonFXMotor(SECOND_MOTOR_ID);

        m_motorGroup = new MotorGroup(m_firstMotor, m_secondMotor);
        m_motorGroup.setVelocityConversionFactor(RPM_TO_RAD_PER_SEC);

        m_encoder = new CANcoder(CAN_CODER_ID);
        m_radSupplier = () -> m_encoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD;

        m_arm = new frc.excalib.mechanisms.Arm.Arm(
                m_motorGroup,
                m_radSupplier,
                new SoftLimit(() -> -MAX_VEL_RAD_PER_SEC, () -> MAX_RAD_LIMIT),
                () -> new Translation2d(1, new Rotation2d(m_radSupplier.getAsDouble())),
                ANGLE_GAINS,
                MASS
        );
        elevatorHeightSupplier = () -> 0;
        this.m_toleranceTrigger = new Trigger(() -> this.isAtTolerance);
        this.m_softLimit = new ContinuousSoftLimit(() ->
                elevatorHeightSupplier.getAsDouble() > ELEVATOR_HEIGHT_LIMIT_TRIGGER ?
                        EXTENDED_MIN_RAD_LIMIT :
                        CLOSED_MIN_RAD_LIMIT, () -> MAX_RAD_LIMIT);
        this.setDefaultCommand(defaultCommand());
    }

    /**
     * @param output
     * @return a command that will run the arm with the given DoubleSupplier
     */
    public Command manualCommand(DoubleSupplier output) {
        return m_arm.manualCommand(output, this);
    }

    public Command changeSetpointCommand(double setpoint) {
        return new InstantCommand(() -> {
            this.setpointAngle = setpoint;
        });
    }

    private Command defaultCommand() {
        return m_arm.anglePositionControlCommand(
                () -> m_softLimit.getSetPoint(m_arm.logPosition(), this.setpointAngle),
                (isAtTolerance) -> {
                    this.isAtTolerance = isAtTolerance;
                },
                TOLERANCE,
                this
        );
    }

    public void setElevatorHeightSupplier(DoubleSupplier elevatorHeightSupplier) {
        this.elevatorHeightSupplier = elevatorHeightSupplier;
    }

}

