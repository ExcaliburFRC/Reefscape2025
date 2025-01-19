package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.arm.Constants.*;
import static java.lang.Math.*;

public class Placer extends SubsystemBase {
    private final TalonFXMotor m_firstRotationMotor = new TalonFXMotor(ANGLE_MOTOR_FIRST_ID),
            m_secondRotationMotor = new TalonFXMotor(ANGLE_MOTOR_SECOND_ID);//motorGroup...

    private final SparkMaxMotor m_scoringMotor, m_intakeMotor;
    private final Mechanism m_scoringWheel, m_intakeWheel;
    private final Arm m_arm;
    private final CANcoder m_angleEncoder;
    private final DigitalInput m_beambrake = new DigitalInput(BEAMBREAK_CHANNEL);
    private final DoubleSupplier m_radSupplier;

    private final MotorGroup motorGroup = new MotorGroup(m_firstRotationMotor, m_secondRotationMotor);

    public Placer() {

        m_scoringMotor = new SparkMaxMotor(SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        m_intakeMotor = new SparkMaxMotor(INTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        m_scoringWheel = new Mechanism(m_scoringMotor);
        m_intakeWheel = new Mechanism(m_intakeMotor);

        m_angleEncoder = new CANcoder(ANGLE_CANCODER_ID);
        m_radSupplier = () -> m_angleEncoder.getPosition().getValueAsDouble() * 2 * PI;
        m_arm = new Arm(
                motorGroup,
                m_radSupplier,
                LIMIT,
                () -> COM_SUPPLIER,
                ANGLE_GAINS,
                MASS
        );

    }

    public Command ManualCommand(DoubleSupplier voltage) {
        Command manualCommand = m_arm.manualCommand(voltage);
        manualCommand.addRequirements(this);
        return manualCommand;
    }
//    public Command goToAngle(DoubleSupplier angle){#TODO Add Tolerance Consumer
//        m_arm.goToAngleCommand(angle.getAsDouble());
//    }
}
