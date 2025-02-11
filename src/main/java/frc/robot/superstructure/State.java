package frc.robot.superstructure;

import static frc.robot.superstructure.Constants.*;

public enum State {
    INTAKE(INTAKE_ELEVATOR_HEIGHT, INTAKE_PLACER_ANGLE, INTAKE_INNER_WHEELS_VOLTAGE, INTAKE_OUT_WHEELS_VOLTAGE),
    L1(L1_ELEVATOR_HEIGHT, L1_PLACER_ANGLE, L1_INNER_WHEELS_VOLTAGE, L1_OUT_WHEELS_VOLTAGE),
    L2(L2_ELEVATOR_HEIGHT, L2_PLACER_ANGLE, L2_INNER_WHEELS_VOLTAGE, L2_OUT_WHEELS_VOLTAGE),
    L3(L3_ELEVATOR_HEIGHT, L3_PLACER_ANGLE, L3_INNER_WHEELS_VOLTAGE, L3_OUT_WHEELS_VOLTAGE),
    L4(L4_ELEVATOR_HEIGHT, L4_PLACER_ANGLE, L4_INNER_WHEELS_VOLTAGE, L4_OUT_WHEELS_VOLTAGE),
    ALGAE2(ALGAE2_ELEVATOR_HEIGHT, ALGAE2_PLACER_ANGLE, ALGAE3_INNER_WHEELS_VOLTAGE, ALGAE2_OUT_WHEELS_VOLTAGE),
    ALGAE3(ALGAE3_ELEVATOR_HEIGHT, ALGAE3_PLACER_ANGLE, ALGAE2_INNER_WHEELS_VOLTAGE, ALGAE3_OUT_WHEELS_VOLTAGE),
    DEFAULT(DEFAULT_ELEVATOR_HEIGHT, DEFAULT_PLACER_ANGLE,DEFAULT_INNER_WHEELS_VOLTAGE,DEFAULT_OUT_WHEELS_VOLTAGE);
    public final double m_elevatorHeight,
            m_placerAngle,
            m_innerWheelsVoltage,
            m_outWheelsVoltage;

    State(
            double elevatorHeight,
            double placerAngle,
            double innerWheelsVoltage,
            double outWheelsVoltage
    ) {
        m_elevatorHeight = elevatorHeight;
        m_placerAngle = placerAngle;
        m_innerWheelsVoltage = innerWheelsVoltage;
        m_outWheelsVoltage = outWheelsVoltage;
    }
}

