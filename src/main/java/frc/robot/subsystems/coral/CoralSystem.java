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
import monologue.Annotations;
import monologue.Logged;

import static edu.wpi.first.wpilibj.I2C.Port.kOnboard;
import static frc.robot.subsystems.coral.Constants.*;
import static monologue.Annotations.*;

public class CoralSystem extends SubsystemBase implements Logged {
    public final Trigger m_hasCoralTrigger;
    private final Mechanism m_coralWheel;
    private final ColorSensorV3 m_colorSensor;
    private double m_voltageState;

    public CoralSystem() {
        this.m_colorSensor = new ColorSensorV3(kOnboard);

        this.m_coralWheel = new Mechanism(new TalonFXMotor(MOTOR_ID));

        this.m_hasCoralTrigger = new Trigger(
                () -> (this.m_coralWheel.logCurrent() > HAS_CORAL_CURRENT) ||
                        (this.m_colorSensor.getProximity() > PROXIMITY_LIMIT)
        ).debounce(HAS_CORAL_DEBOUNCE);

        this.m_voltageState = 0;
        this.setDefaultCommand(defaultCommand());
    }

    public Command manualCommand(double voltage) {
        return m_coralWheel.manualCommand(() -> voltage, this).withName("Manual Command");
    }

    private Command defaultCommand() {
        return new RunCommand(
                () -> this.m_coralWheel.setVoltage(this.m_voltageState),
                this
        ).withName("Default Command");
    }

    public Command setStateCommand(double voltageState) {
        return new RunCommand(() -> this.m_voltageState = voltageState).until(() -> m_voltageState == voltageState);
    }

    @Log.NT
    public boolean hasCoral() {
        return m_hasCoralTrigger.getAsBoolean();
    }

    @Log.NT
    public int getProximity() {
        return m_colorSensor.getProximity();
    }
}
