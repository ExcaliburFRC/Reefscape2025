package frc.excalib.mechanisms.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * This class represents an Arm Mechanism
 */
public class Arm extends Mechanism {
    private final Mass m_mass;
    private final PIDController m_PIDController;
    public final DoubleSupplier ANGLE_SUPPLIER;
    public final double m_kv, m_ks, m_kg;
    public final SoftLimit m_VELOCITY_LIMIT;

    public Arm(Motor motor,
               DoubleSupplier angleSupplier,
               SoftLimit velocityLimit,
               Supplier<Translation2d> comSupplier,
               Gains gains,
               double mass) {
        super(motor);
        ANGLE_SUPPLIER = angleSupplier;
        m_VELOCITY_LIMIT = velocityLimit;
        m_kg = gains.kg;
        m_kv = gains.kv;
        m_ks = gains.ks;
        m_PIDController = new PIDController(gains.kp, gains.ki, gains.kd);
        m_mass = new Mass(() -> comSupplier.get().getX(), () -> comSupplier.get().getY(), mass);
    }

    /**
     * @param setPointSupplier the dynamic angle setpoint to go to (radians)
     * @param toleranceConsumer gets updated if the measurement is at tolerance.
     * @return a command that moves the arm to the specified dynamic setpoint.
     */
    public Command anglePositionControlCommand(DoubleSupplier setPointSupplier, Consumer<Boolean> toleranceConsumer) {
        final double dutyCycle = 0.02;
        return new RunCommand(() -> {
            double error = setPointSupplier.getAsDouble() - ANGLE_SUPPLIER.getAsDouble();
            double velocitySetpoint = error / dutyCycle;
            velocitySetpoint = m_VELOCITY_LIMIT.limit(velocitySetpoint);
            double phyOutput =
                    m_ks * Math.signum(velocitySetpoint) +
                            m_kg * m_mass.getCenterOfMass().getX();
            double pid = m_PIDController.calculate(ANGLE_SUPPLIER.getAsDouble(), setPointSupplier.getAsDouble());
            double output = phyOutput + pid;
            super.setVoltage(output);
            toleranceConsumer.accept(Math.abs(error) < 0.2);
        });
    }

    /**
     * @param angle the angle setpoint to go to (radians)
     * @param toleranceConsumer gets updated if the measurement is at tolerance.
     * @return a command that moves the arm to the specified setpoint.
     */
    public Command goToAngleCommand(double angle, Consumer<Boolean> toleranceConsumer) {
        return anglePositionControlCommand(() -> angle, toleranceConsumer);
    }
}