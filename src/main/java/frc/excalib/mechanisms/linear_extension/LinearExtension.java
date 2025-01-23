package frc.excalib.mechanisms.linear_extension;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Mechanism;
import frc.excalib.control.gains.Gains;

import java.util.function.DoubleSupplier;

public class LinearExtension extends Mechanism {
    private final DoubleSupplier m_positionSupplier;
    private final DoubleSupplier m_angleSupplier;
    private final PIDController m_PIDController;
    private final Gains m_gains;


    private final double m_MAX_VELOCITY, m_MAX_ACCELERATION;

    public LinearExtension(Motor motor, DoubleSupplier positionSupplier, DoubleSupplier angleSupplier, double maxAcceleration, double maxVelocity, Gains gains) {
        super(motor);
        m_MAX_ACCELERATION = maxAcceleration;
        m_MAX_VELOCITY = maxVelocity;
        m_positionSupplier = positionSupplier;
        m_angleSupplier = angleSupplier;
        m_gains = gains;
        m_PIDController = new PIDController(gains.kp, gains.ki, gains.kd);


    }

    public Command extendCommand(DoubleSupplier lengthSetPoint) {
        return new TrapezoidProfileCommand(
                new TrapezoidProfile(new TrapezoidProfile.Constraints(m_MAX_VELOCITY, m_MAX_ACCELERATION)),
                state -> {
                    double pidValue = m_PIDController.calculate(super.m_motor.getMotorPosition(), state.position);
                    double ff =
                            m_gains.ks * Math.signum(state.velocity) +
                                    m_gains.kv * state.velocity +
                                    m_gains.kg * Math.cos(m_angleSupplier.getAsDouble());
                    setVoltage(ff + pidValue);
                    },
                () -> new TrapezoidProfile.State(lengthSetPoint.getAsDouble(), 0),
                () -> new TrapezoidProfile.State(super.m_motor.getMotorPosition(), super.m_motor.getMotorVelocity())

        );
    }

    public double logVoltage(){
        return m_motor.getVoltage();
    }
    public double logVelocity(){
        return m_motor.getMotorVelocity();
    }
    public double logPosition(){
        return m_motor.getMotorPosition();
    }

}