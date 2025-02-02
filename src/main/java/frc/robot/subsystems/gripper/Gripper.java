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
    private final SparkMaxMotor m_scoringMotor, m_intakeMotor;
    private final Mechanism m_scoringWheel, m_intakeWheel;
    private final DigitalInput m_beambrake = new DigitalInput(BEAMBREAK_CHANNEL);//TODO
    public final Trigger hasCoralTrigger = new Trigger(m_beambrake::get);

    public Gripper() {
        m_scoringMotor = new SparkMaxMotor(SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        m_intakeMotor = new SparkMaxMotor(INTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        m_scoringWheel = new Mechanism(m_scoringMotor);
        m_intakeWheel = new Mechanism(m_intakeMotor);

    }

    public Command manualCommand(double scoringVoltage, double intakeVoltage) {
        return new ParallelCommandGroup(new RunCommand(() -> m_intakeWheel.setVoltage(intakeVoltage)), new RunCommand(() -> m_scoringWheel.setVoltage(scoringVoltage)));
    }
}
