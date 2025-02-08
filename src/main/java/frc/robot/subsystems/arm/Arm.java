package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
        m_radSupplier = () -> m_angleEncoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD;
        m_arm = new frc.excalib.mechanisms.Arm.Arm(
                m_motorGroup,
                m_radSupplier,
                LIMIT,
                () -> new Translation2d(
                        1,
                        new Rotation2d(m_radSupplier.getAsDouble()
                        )),
                ANGLE_GAINS,
                MASS
        );
        this.toleranceTrigger = new Trigger(() -> this.isAtTolerance);
        this.setDefaultCommand(goToSetPointAngleCommand());
    }

    /**
     * @param armDC
     * @return
     */
    public Command manualCommand(DoubleSupplier armDC) {
        return m_arm.manualCommand(armDC, this);
    }

    public Command changeSetPointCommand(double setPoint) {
        return new InstantCommand(() -> {
            this.setpointAngle = setPoint;
        });
    }

    public Command goToSetPointAngleCommand() {
        return m_arm.anglePositionControlCommand(() -> this.setpointAngle, (isAtTolerance) -> {
            this.isAtTolerance = isAtTolerance;
        }, MAX_OFFSET, this);
    }
}

