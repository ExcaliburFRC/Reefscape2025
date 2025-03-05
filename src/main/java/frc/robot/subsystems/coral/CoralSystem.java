package frc.robot.subsystems.coral;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;

public class CoralSystem extends SubsystemBase {
    private final Trigger m_hasCoral;
    private final Mechanism m_coralWheel;
    private double voltageState;
    private final ColorSensorV3 m_colorSensor;

    public CoralSystem() {
        this.m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        this.m_coralWheel = new Mechanism(new TalonFXMotor(0));
        this.m_hasCoral = new Trigger(
                () -> this.m_coralWheel.logCurrent() > 0 ||
                        this.m_colorSensor.getProximity() > 80
        ).debounce(0);
        this.voltageState = 0;
        this.setDefaultCommand(defaultCommand());
    }

    private Command defaultCommand() {
        return new RunCommand(
                () -> {
                    this.m_coralWheel.setVoltage(this.voltageState);
                },
                this
        );
    }

    public Command setStateCommand(double voltageState) {
        return new InstantCommand(() -> this.voltageState = voltageState);
    }
}