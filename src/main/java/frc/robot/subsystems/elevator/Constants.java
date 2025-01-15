package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;

public class Constants {
    //IDs
    static final int MOTOR_ID = 0; //TODO
    static final int LIMIT_SWITCH_CHANNEL = 0; //TODO

    //Measurements
    static final double ELEVATOR_ANGLE = Math.PI/2;

    //Gains and Constrains
    private static final double kp = 0;
    private static final double ki = 0;
    private static final double kd = 0;
    private static final double ks = 0;
    private static final double ka = 0;
    private static final double kv = 0;
    private static final double kg = 0;
    static final Gains ELEVATOR_GAINS = new Gains(kp, ki,kd, ks, kv, ka, kg);

    static final TrapezoidProfile.Constraints UPWARD_CONSTAINTS =
            new TrapezoidProfile.Constraints(
                    0,
                    0
            ); //TODO

    static final TrapezoidProfile.Constraints DOWNWARD_CONSTAINTS =
            new TrapezoidProfile.Constraints(
                    0,
                    0
            ); //TODO
}
