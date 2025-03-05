package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Mechanism;

public class AlgaeSystem extends SubsystemBase {
    private final Trigger m_hasAlgae;
    private final Mechanism m_algaeWheel;
    private double voltageState;

    public AlgaeSystem() {
        this.m_algaeWheel = new Mechanism(new TalonFXMotor(0));
        this.m_hasAlgae = new Trigger(() -> this.m_algaeWheel.logCurrent() > 0).debounce(0);
       this.voltageState = 0;
        this.setDefaultCommand(defaultCommand());
    }


    private Command defaultCommand() {
        return new RunCommand(
                () -> {
                    this.m_algaeWheel.setVoltage(this.voltageState);
                },
                this
        );
    }

    public Command setStateCommand(double voltageState) {
        return new InstantCommand(() -> this.voltageState = voltageState);
    }
}
