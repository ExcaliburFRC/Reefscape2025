package frc.robot.subsystems.arm;

import frc.excalib.control.gains.Gains;

public class Constants {
    //IDS and Channels
    public static final int FIRST_MOTOR_ID = 30;
    public static final int SECOND_MOTOR_ID = 31;
    public static final int CAN_CODER_ID = 32;

    //Gains for PID and FF
    private static final double kp = 6.5;
    private static final double ki = 0;
    private static final double kd = 0.1;
    private static final double ka = 0;
    private static final double kv = 0;
    private static final double kg = 0.3;
    private static final double ks = 0;
    public static final Gains ANGLE_GAINS = new Gains(kp, ki, kd, ks, kv, ka, kg);

    //Physical Properties
    public static final double MASS = 1;
    public static final double TOLERANCE = 0.07;
    public static final double ROTATIONS_TO_RAD = -2 * Math.PI;
    public static final double POSITION_CONVERSION_FACTOR = -1 / 7.5498750273499192965754627065202;
    public static final double RPS_TO_RAD_PER_SEC = 1 / 7.5498750273499192965754627065202;

    public static final double MAX_RAD_LIMIT = 2.0586022173425302;
    public static final double MIN_RAD_LIMIT = -1.966563370069392;
    public static final double MIN_ALGAE_SHAFT_LIMIT = -1.4496118445519308;
    public static final double MIN_ALGAE_SHIELD_LIMIT = -1.2624661884298827;
    public static final double MIN_PROFILE_LIMIT = -0.9495341077012119;
    public static final double MIN_PROFILE_CORAL_LIMIT = -0.6703496043060252;

    public static final double ELEVATOR_LOWER_PROFILE_CORAL_TRIGGER = 0.2484445461082967;
    public static final double ELEVATOR_UPPER_PROFILE_CORAL_TRIGGER = 0.5879150367595152;
    public static final double ELEVATOR_UPPER_PROFILE_TRIGGER = 0.5879150367595152;
    public static final double ELEVATOR_LOWER_PROFILE_TRIGGER = 0.26;
    public static final double ELEVATOR_ALGAE_SHIELD_TRIGGER = 0.12;

    public static double OPENED_MAX_VOL = 1;
    public static double CLOSED_MAX_VOL = 6;
}
