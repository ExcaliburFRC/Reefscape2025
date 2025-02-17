package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;

public class Constants {
    //IDs
    static final int FIRST_MOTOR_ID = 20;
    static final int SECOND_MOTOR_ID = 21;

    //Measurements
    static final double ELEVATOR_ANGLE = Math.PI/2;
    static final double TOLERANCE = 0.01;
    static final double ROTATIONS_TO_METERS = 0.011767896627459766;

    //Gains and Constrains
    private static final double kp = 0; //TODO
    private static final double ki = 0; //TODO
    private static final double kd = 0; //TODO
    private static final double ks = 0.11663;
    private static final double ka = 0; //TODO
    private static final double kv = 10.224;
    private static final double kg = 0.18203;
    static final Gains ELEVATOR_GAINS = new Gains(kp, ki,kd, ks, kv, ka, kg);

    static final double MIN_HEIGHT = 0;
    static final double STALL_THRESHOLD = 1;
    static final double MAX_HEIGHT = 0.7;
    static final double LOWER_MAX_HEIGHT = 0.2; //TODO
    static final double UPPER_MIN_HEIGHT = 0.46; //TODO
    static final double ARM_COLLISION_RAD = 0; //TODO: get arm defualt angle

    static final TrapezoidProfile.Constraints CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                    1.1,
                    5
            );

}
