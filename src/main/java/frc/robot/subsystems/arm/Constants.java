package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;

public class Constants {
    public static final int ANGLE_MOTOR_FIRST_ID = 0;//TODO
    public static final int ANGLE_MOTOR_SECOND_ID = 0;//TODO
    public static  final int INTAKE_MOTOR_ID = 0;//TODO
    public static final int SHOOTER_MOTOR_ID = 0;//TODO
    public static final int ANGLE_CANCODER_ID = 0;//TODO
    public static final int LEFT_MOTOR_ID = 0;//TODO
    public static final int RIGHT_MOTOR_ID=0;//TODO
    public static final int BEAMBREAK_CHANNEL = 0;//TODO
    public static final SoftLimit LIMIT = new SoftLimit(()->0,()->0);
    public static final Translation2d COM_SUPPLIER = new Translation2d(0,0);

    //Gains
    private static final int kp = 0;
    private static final int ki = 0;
    private static final int kd = 0;
    private static final int ka = 0;
    private static final int kv = 0;
    private static final int kg = 0;
    private static final int ks = 0;
    public static final double MASS = 0;
    public static final Gains ANGLE_GAINS = new Gains(kp, ki, kd, ks, kv, ka, kg);
    public static final double CENTER_OF_MASS_RADIUS = 0;
    public static final double CENTER_OF_MASS_OFFSET_RAD=0;
    public static final double MAX_OFFSET = 0.2;

}
