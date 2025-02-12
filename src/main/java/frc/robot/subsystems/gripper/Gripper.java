package frc.robot.subsystems.gripper;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.FlexMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;

import static frc.robot.subsystems.gripper.Constants.*;

public class Gripper extends SubsystemBase {
    private final FlexMotor m_outerMotor;
    private final TalonFXMotor m_innerMotor;
    private final Mechanism m_outerWheel, m_innerWheel;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    public final Trigger m_coralTrigger = new Trigger(()->m_colorSensor.getProximity()>PROXIMITY_LIMIT);

    public Gripper() {
        m_outerMotor = new FlexMotor(OUTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        m_innerMotor = new TalonFXMotor(INNER_MOTOR_ID);
        m_outerWheel = new Mechanism(m_outerMotor);
        m_innerWheel = new Mechanism(m_innerMotor);

    }

    public Command manualCommand(double innerVoltage, double outerVoltage) {
        return new ParallelCommandGroup(
                new RunCommand(
                        () -> m_innerWheel.setVoltage(innerVoltage)
                ),
                new RunCommand(
                        () -> m_outerWheel.setVoltage(outerVoltage)
                )
        );
    }
}
