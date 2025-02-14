package frc.robot.subsystems.arm;

import frc.excalib.control.gains.Gains;

public class Constants {
    //IDS and Channels
    public static final int FIRST_MOTOR_ID = 30;
    public static final int SECOND_MOTOR_ID = 31;
    public static final int CAN_CODER_ID = 32;

    //Gains for PID and FF
    private static final double kp = 7;
    private static final double ki = 0; // TODO
    private static final double kd = 0; // TODO
    private static final double ka = 0; // TODO
    private static final double kv = 1.6548; // TODO
    private static final double kg = 0.3;
    private static final double ks = 0; // TODO
    public static final Gains ANGLE_GAINS = new Gains(kp, ki, kd, ks, kv, ka, kg);


    //Physical Proprieties
    public static final double MASS = 1;
    public static final double TOLERANCE = 0.02; // TODO
    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;
    private static final double CONVERSION = 55.56;
    public static final double POSITION_CONVERSION_FACTOR = 1 / 7.5498750273499192965754627065202;
    public static final double RPS_TO_RAD_PER_SEC = 1 / 7.5498750273499192965754627065202;
    public static double ELEVATOR_HEIGHT_LIMIT_TRIGGER = 0.36;
    public static double MAX_RAD_LIMIT = 0;
    public static double EXTENDED_MIN_RAD_LIMIT = 0;
    public static double CLOSED_MIN_RAD_LIMIT = 0;
    public static double MAX_VOL_RAD_PER_SEC = 3.5;
}
