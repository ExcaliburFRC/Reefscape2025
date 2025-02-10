package frc.robot.subsystems.gripper;

import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.mechanisms.Mechanism;

import static frc.robot.subsystems.arm.Constants.*;

public class Gripper extends SubsystemBase {
    private final SparkMaxMotor m_outerMotor, m_innerMotor;
    private final Mechanism m_outerWheel, m_innerWheel;
    private final DigitalInput m_beambrake = new DigitalInput(BEAMBREAK_CHANNEL);
    public final Trigger m_coralTrigger = new Trigger(m_beambrake::get);

    public Gripper() {
        m_outerMotor = new SparkMaxMotor(SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        m_innerMotor = new SparkMaxMotor(INTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        m_outerWheel = new Mechanism(m_outerMotor);
        m_innerWheel = new Mechanism(m_innerMotor);

    }

    public Command manualCommand(double innerVoltage, double outerVoltage) {
        return new ParallelCommandGroup(
                new RunCommand(
                        () -> m_innerWheel.setVoltage(outerVoltage)
                ),
                new RunCommand(
                        () -> m_outerWheel.setVoltage(innerVoltage)
                )
        );
    }
}
