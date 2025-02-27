package frc.robot.subsystems.gripper;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.FlexMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import frc.robot.superstructure.State;
import monologue.Logged;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.I2C.Port.kOnboard;
import static frc.robot.subsystems.gripper.Constants.*;
import static monologue.Annotations.Log;

public class Gripper extends SubsystemBase implements Logged {
    private final FlexMotor m_outerMotor;
    private final TalonFXMotor m_innerMotor;
    private final Mechanism m_outerWheel, m_innerWheel;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(kOnboard);
    public final Trigger m_coralTrigger = new Trigger(() -> m_colorSensor.getProximity() > PROXIMITY_LIMIT).debounce(0.05);
    private double innerVoltage, outerVoltage;

    public Gripper() {
        m_outerMotor = new FlexMotor(OUTER_MOTOR_ID, kBrushless);
        m_outerMotor.setCurrentLimit(0, 25);
        m_innerMotor = new TalonFXMotor(INNER_MOTOR_ID);
        m_innerMotor.setCurrentLimit(0, 70);
        m_outerWheel = new Mechanism(m_outerMotor);
        m_innerWheel = new Mechanism(m_innerMotor);
        innerVoltage = 0;
        outerVoltage = 0;
        this.setDefaultCommand(defaultCommand());
        SmartDashboard.putData(this);
    }

    public Command manualCommand(double innerVoltage, double outerVoltage) {
        Command manualCommand = new ParallelCommandGroup(
                new RunCommand(() -> m_innerWheel.setVoltage(innerVoltage)),
                new RunCommand(() -> m_outerWheel.setVoltage(outerVoltage))
        ).withName("Manual Command");
        manualCommand.addRequirements(this);
        return manualCommand;
    }

    private Command defaultCommand() {
        Command defaultCommand = new ParallelCommandGroup(
                new RunCommand(() -> {
                    m_innerWheel.setVoltage(innerVoltage);
                }),
                new RunCommand(() -> {
                    m_outerWheel.setVoltage(outerVoltage);
                })
        );
        defaultCommand.addRequirements(this);
        return defaultCommand;
    }

    public Command setStateCommand(double innerVoltage, double outerVoltage) {
        return new InstantCommand(
                () -> {
                    this.innerVoltage = innerVoltage;
                    this.outerVoltage = outerVoltage;
                }
        );
    }

    @Log.NT
    public boolean hasCoral() {
        return m_coralTrigger.getAsBoolean();
    }
}
