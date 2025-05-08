package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import monologue.Logged;

import static frc.robot.subsystems.algae.Constants.*;
import static monologue.Annotations.*;

public class AlgaeSystem extends SubsystemBase implements Logged {
    private final Mechanism m_algaeWheel;
    public final Trigger m_hasAlgaeTrigger;
    private double m_voltageState;

    public AlgaeSystem() {
        TalonFXMotor m_motor = new TalonFXMotor(MOTOR_ID);
        m_motor.setCurrentLimit(25,40);

        this.m_algaeWheel = new Mechanism(m_motor);

        this.m_hasAlgaeTrigger = new Trigger(
                () -> (this.m_algaeWheel.logCurrent() > Math.abs(HAS_ALGAE_CURRENT)) &&
                        (Math.abs(m_algaeWheel.logVelocity()) < HAS_ALGAE_VELOCITY)).debounce(HAS_ALGAE_DEBOUNCE);

        this.m_voltageState = 0;
        this.setDefaultCommand(defaultCommand());
    }

    public Command manualCommand(double voltage) {
        return m_algaeWheel.manualCommand(() -> voltage, this);
    }

    private Command defaultCommand() {
        return new RunCommand(() -> this.m_algaeWheel.setVoltage(this.m_voltageState), this);
    }

    public Command setStateCommand(double voltageState) {
        return new RunCommand(() -> this.m_voltageState = voltageState).until(() -> m_voltageState == voltageState);
    }

    @Log.NT
    public boolean hasAlgae() {
        return m_hasAlgaeTrigger.getAsBoolean();
    }
}
