package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;
import monologue.Annotations;

import static frc.robot.subsystems.algae.Constants.*;
import static monologue.Annotations.*;

public class AlgaeSystem extends SubsystemBase {
    private final Trigger m_hasAlgaeTrigger;
    private final Mechanism m_algaeWheel;
    private double m_voltageState;

    public AlgaeSystem() {
        this.m_algaeWheel = new Mechanism(new TalonFXMotor(MOTOR_ID));

        this.m_hasAlgaeTrigger = new Trigger(
                () -> this.m_algaeWheel.logCurrent() > HAS_ALGAE_CURRENT
        ).debounce(HAS_ALGAE_DEBOUNCE);

        this.m_voltageState = 0;
        this.setDefaultCommand(defaultCommand());
    }

    public Command manualCommand(double voltage) {
        return m_algaeWheel.manualCommand(() -> voltage, this);
    }

    private Command defaultCommand() {
        return new RunCommand(
                () -> {
                    this.m_algaeWheel.setVoltage(this.m_voltageState);
                },
                this
        );
    }

    public Command setStateCommand(double voltageState) {
        return new RunCommand(() -> this.m_voltageState = voltageState).until(() -> m_voltageState == voltageState);
    }

    @Log.NT
    public boolean hasAlgae() {
        return m_hasAlgaeTrigger.getAsBoolean();
    }
}
