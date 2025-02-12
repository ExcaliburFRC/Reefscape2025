package frc.excalib.control.motor.controllers;

import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;

public interface Motor {
    void stopMotor();

    void setVoltage(double voltage);

    void setPercentage(double percentage);

    void setFollower(int mainMotorID);

    void setIdleState(IdleState idleMode);

    IdleState getIdleState();

    int getDeviceID();

    double getMotorPosition();

    double getMotorVelocity();

    double getCurrent();

    double getVoltage();

    double getTemperature();

    void setSoftLimit(DirectionState directionState, float limit);

    void setInverted(DirectionState mode);

    void setPositionConversionFactor(double conversionFactor);

    void setVelocityConversionFactor(double conversionFactor);

    void setCurrentLimit(int stallLimit, int freeLimit);

    void setMotorPosition(double position);
}
