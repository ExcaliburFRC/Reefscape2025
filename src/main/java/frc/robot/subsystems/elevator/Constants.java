package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;

public class Constants {
    //IDs
    static final int FIRST_MOTOR_ID = 20;
    static final int SECOND_MOTOR_ID = 21;

    //Measurements
    static final double ELEVATOR_ANGLE = Math.PI / 2;
    static final double TOLERANCE = 0.025;
    static final double ROTATIONS_TO_METERS = 0.011767896627459766;

    //Gains and Constrains
    private static final double kp = 0; //TODO
    private static final double ki = 0; //TODO
    private static final double kd = 0; //TODO
    private static final double ks = 0.038911;
    private static final double ka = 0; //TODO
    private static final double kv = 10.05;
    private static final double kg = 0.16602;
    static final Gains ELEVATOR_GAINS = new Gains(kp, ki, kd, ks, kv, ka, kg);

    static final double STALL_THRESHOLD = 1;
    static final double MIN_HEIGHT_LIMIT = 0.01;
    static final double MAX_HEIGHT_LIMIT = 0.69;
    static final double ARM_COLLISION_TRIGGER = -1.0967962633382335;
    static final double CORAL_COLLISION_TRIGGER = -0.760854470791278;
    static final double LOWER_MAX_LIMIT = 0.2727273249889827;
    static final double UPPER_MIN_LIMIT = 0.5495056104869298;

    static final TrapezoidProfile.Constraints CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                    1.1,
                    5
            );
}
