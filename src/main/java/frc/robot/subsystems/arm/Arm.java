package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.arm.Constants.*;
import static java.lang.Math.*;

public class Arm extends SubsystemBase {
    private final TalonFXMotor m_firstRotationMotor, m_secondRotationMotor;
    private final MotorGroup m_motorGroup;
    private final frc.excalib.mechanisms.Arm.Arm m_arm;
    private final CANcoder m_angleEncoder;
    private final DoubleSupplier m_radSupplier;
    private boolean isAtTolerance = false;
    public final Trigger toleranceTrigger;
    private double setpointAngle = 0;

    public Arm() {
        m_firstRotationMotor = new TalonFXMotor(ANGLE_MOTOR_FIRST_ID);
        m_secondRotationMotor = new TalonFXMotor(ANGLE_MOTOR_SECOND_ID);

        m_motorGroup = new MotorGroup(m_firstRotationMotor, m_secondRotationMotor);

        m_angleEncoder = new CANcoder(ANGLE_CANCODER_ID);
        m_radSupplier = () -> m_angleEncoder.getPosition().getValueAsDouble() * 2 * PI;
        m_arm = new frc.excalib.mechanisms.Arm.Arm(
                m_motorGroup,
                m_radSupplier,
                LIMIT,
                () -> COM_SUPPLIER,
                ANGLE_GAINS,
                MASS
        );
        this.toleranceTrigger = new Trigger(() -> this.isAtTolerance);
        this.setDefaultCommand(goToSetpointAngleCommand());
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

    public Command goToSetpointAngleCommand() {
        return m_arm.anglePositionControlCommand(
                () -> this.setpointAngle,
                (isAtTolerance) -> {
                    this.isAtTolerance = isAtTolerance;
                },
                MAX_OFFSET,
                this
        );
    }
}

