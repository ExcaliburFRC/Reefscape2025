package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;

public class Constants {
    //IDS and Channels
    public static final int ANGLE_MOTOR_FIRST_ID = 0;//TODO
    public static final int ANGLE_MOTOR_SECOND_ID = 0;//TODO
    public static final int INTAKE_MOTOR_ID = 0;//TODO
    public static final int SHOOTER_MOTOR_ID = 0;//TODO
    public static final int ANGLE_CANCODER_ID = 0;//TODO
    public static final int LEFT_MOTOR_ID = 0;//TODO
    public static final int RIGHT_MOTOR_ID = 0;//TODO
    public static final int BEAMBREAK_CHANNEL = 0;//TODO

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
    public static final double MAX_OFFSET = 0; // TODO
    private static final double MIN_LIMIT = 0.0; // TODO
    private static final double MAX_LIMIT = 0.0; // TODO
    public static final Translation2d COM_SUPPLIER = new Translation2d(0, 0);
    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;

    //Limits
    public static final SoftLimit LIMIT = new SoftLimit(() -> MIN_LIMIT, () -> MAX_LIMIT);

    //Velocities and DutyCycles
    public static final double ALGAE_REMOVAL_DC = 0; //TODO
    public static final double INTAKE_CORAL_DC = 0; //TODO
    public static final double OUTPUT_CORAL_DC = 0; //TODO

}
