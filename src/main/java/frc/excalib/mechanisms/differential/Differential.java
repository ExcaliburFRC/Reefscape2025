package frc.excalib.mechanisms.differential;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.control.motor.motor_types.DifferentialMotor;
import frc.excalib.mechanisms.Mechanism;
import monologue.Annotations.Log;

import java.util.function.DoubleSupplier;

public class Differential extends Mechanism {
    private final Gains gains;
    private final PIDController pidController_A, pidController_B;

    private final DifferentialMotor motor;

    public final Trigger atSetpointTrigger;

    private final double differentialMul;

    // motor's delta from setpoint position
    private double deltaA, deltaB;
    // motor's current position
    private double motorApos, motorBpos;

//    private DoubleSupplier angleSetpointSupplier, linearSetpointSupplier;
//    DoubleSupplier angleSetpointSupplier, DoubleSupplier linearSetpointSupplier

    /**
     * @param differentialMotor a differential motor object of both mechanism motors
     * @param gains gains for pid & ff for the two motors
     * @param differentialMul an optional multiplier to correct acc error in certain types of differential mechanism
     * @param tolerance tolerance for the accumulated error from BOTH motors combined
     */
    public Differential(DifferentialMotor differentialMotor, Gains gains, double differentialMul, double tolerance) {
        super(differentialMotor);

        this.motor = differentialMotor;
        this.motor.setIdleState(IdleState.BRAKE);

        this.gains = gains;
        this.atSetpointTrigger = new Trigger(()-> (Math.abs(deltaA - motorApos) + Math.abs(deltaB - motorBpos)) < tolerance);

        this.pidController_A = this.gains.getPIDcontroller();
        this.pidController_B = this.gains.getPIDcontroller();

        this.differentialMul = differentialMul;
    }

    /**
     * constructor for a manual diff, w/o advanced control capabilities
     * @param differentialMotor
     */
    public Differential(DifferentialMotor differentialMotor) {
        this(differentialMotor, new Gains(), 1, 20);
    }

    public Command moveToStateCommand(double angle, double position, double ff, SubsystemBase... requirements){
        return Commands.startRun(
                ()-> {
                    double deltaTheta = angle - getMechanismAngle();
                    double deltaPos = position - getMechanismPosition();

                    deltaA = deltaTheta + (deltaPos / (2 * differentialMul));
                    deltaB = deltaTheta - (deltaPos / (2 * differentialMul));
                },
                ()-> setDifferentialVoltage(
                        pidController_A.calculate(motorApos, this.deltaA) + ff,
                        pidController_B.calculate(motorBpos, this.deltaB) - ff
                ),
                requirements);
    }

    public Command setDifferentialVoltageCommand(DoubleSupplier voltageA, DoubleSupplier voltageB, SubsystemBase... requirements){
        return new RunCommand(
                ()-> setDifferentialVoltage(voltageA.getAsDouble(), voltageB.getAsDouble()),
                requirements);
    }

    private void move(double voltage) {
        super.setDifferentialVoltage(voltage, -voltage);
    }

    private void rotate(double voltage) {
        super.setDifferentialVoltage(voltage, voltage);
    }

    protected void setDifferentialVoltage(double voltageA, double voltageB) {
        super.setDifferentialVoltage(voltageA, voltageB);
    }

    public void periodic(){
        this.motorApos = this.motor.getMotorPositions().getFirst();
        this.motorBpos = this.motor.getMotorPositions().getSecond();
    }

    @Log.NT
    public double getMechanismAngle() {
        return this.motor.getMotorDifference();
    }

    @Log.NT
    public double getMechanismPosition(){
        return this.differentialMul * this.motor.getMotorDifference();
    }
}