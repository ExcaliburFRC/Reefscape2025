package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import monologue.Logged;

import static frc.robot.subsystems.coral.Constants.*;
import static monologue.Annotations.*;

public class CoralSystem extends SubsystemBase implements Logged {
    public final Trigger m_hasCoralTrigger;
    public final DigitalInput m_limitSwitch = new DigitalInput(BUMPER_SWITCH_ID);
    boolean hasCoral = false;
    private final Mechanism m_coralWheel;
    private double m_voltageState;

    public CoralSystem() {
        this.m_coralWheel = new Mechanism(new TalonFXMotor(MOTOR_ID));

//        this.m_hasCoralTrigger = new Trigger(
//                () -> hasCoral
//        ).debounce(HAS_CORAL_DEBOUNCE);

        this.m_hasCoralTrigger = new Trigger(() -> !m_limitSwitch.get()).debounce(HAS_CORAL_DEBOUNCE);
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
}
