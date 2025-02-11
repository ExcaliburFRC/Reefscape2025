package frc.robot.subsystems.arm;

import frc.excalib.control.gains.Gains;

public class Constants {
    //IDS and Channels
    public static final int FIRST_MOTOR_ID = 0;//TODO
    public static final int SECOND_MOTOR_ID = 0;//TODO
    public static final int CAN_CODER_ID = 0;

    //Gains for PID and FF
    private static final int kp = 0; // TODO
    private static final int ki = 0; // TODO
    private static final int kd = 0; // TODO
    private static final int ka = 0; // TODO
    private static final int kv = 0; // TODO
    private static final int kg = 0; // TODO
    private static final int ks = 0; // TODO
    public static final Gains ANGLE_GAINS = new Gains(kp, ki, kd, ks, kv, ka, kg);


    //Physical Proprieties
    public static final double MASS = 1;
    public static final double TOLERANCE = 0; // TODO
    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;
    private static final double CONVERSION = 0;
    public static final double RPM_TO_RAD_PER_SEC = CONVERSION * ROTATIONS_TO_RAD / 60.0;
    public static double ELEVATOR_HEIGHT_LIMIT_TRIGGER = 0;
    public static double MAX_RAD_LIMIT = 0;
    public static double EXTENDED_MIN_RAD_LIMIT = 0;
    public static double CLOSED_MIN_RAD_LIMIT = 0;
    public static double MAX_VEL_RAD_PER_SEC = 0;
}
