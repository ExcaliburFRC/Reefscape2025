// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.additional_utilities.AllianceUtils.AlliancePose;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.imu.IMU;
import frc.excalib.control.imu.Pigeon;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.swerve.ModulesHolder;
import frc.excalib.swerve.Swerve;
import frc.excalib.swerve.SwerveModule;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class SwerveConstants {
        public static final int FRONT_LEFT_DRIVE_ID = 11;
        public static final int FRONT_RIGHT_DRIVE_ID = 13;
        public static final int BACK_RIGHT_DRIVE_ID = 15;
        public static final int BACK_LEFT_DRIVE_ID = 17;

        public static final int FRONT_LEFT_ROTATION_ID = 10;
        public static final int FRONT_RIGHT_ROTATION_ID = 11;
        public static final int BACK_RIGHT_ROTATION_ID = 12;
        public static final int BACK_LEFT_ROTATION_ID = 13;

        public static final int GYRO_ID = 50;
        public static final String SWERVE_CANBUS = "CTRESwerve";

        private static final double PID_TOLERANCE = 0.2; //TODO

        public static final double TRACK_WIDTH = 0.57; // m
        public static final Translation2d FRONT_LEFT_TRANSLATION =
                new Translation2d(
                        TRACK_WIDTH / 2, TRACK_WIDTH / 2
                );
        public static final Translation2d FRONT_RIGHT_TRANSLATION =
                new Translation2d(
                        TRACK_WIDTH / 2, -TRACK_WIDTH / 2
                );
        public static final Translation2d BACK_LEFT_TRANSLATION =
                new Translation2d(
                        -TRACK_WIDTH / 2, TRACK_WIDTH / 2
                );
        public static final Translation2d BACK_RIGHT_TRANSLATION =
                new Translation2d(
                        -TRACK_WIDTH / 2, -TRACK_WIDTH / 2
                );

        public static final double MAX_MODULE_VEL = 4.45; //TODO
        public static final double MAX_FRONT_ACC = 2; //TODO
        public static final double MAX_SIDE_ACC = 6; //TODO
        public static final double MAX_SKID_ACC = 9; //TODO
        public static final double MAX_FORWARD_ACC = 9; //TODO
        public static final double MAX_VEL = 4.5; //TODO
        public static final double MAX_OMEGA_RAD_PER_SEC = 4; //11.2 //TODO
        public static final double MAX_OMEGA_RAD_PER_SEC_SQUARE = 1; //TODO

        public static final PathConstraints MAX_PATH_CONSTRAINTS = new PathConstraints(
                MAX_VEL,
                MAX_SKID_ACC,
                MAX_OMEGA_RAD_PER_SEC,
                MAX_OMEGA_RAD_PER_SEC_SQUARE,
                12.0,
                false
        );

        private static final CANcoder FRONT_LEFT_ABS_ENCODER = new CANcoder(10, SWERVE_CANBUS);
        public static final CANcoder FRONT_RIGHT_ABS_ENCODER = new CANcoder(12, SWERVE_CANBUS);
        private static final CANcoder BACK_RIGHT_ABS_ENCODER = new CANcoder(14, SWERVE_CANBUS);
        private static final CANcoder BACK_LEFT_ABS_ENCODER = new CANcoder(16, SWERVE_CANBUS);

        private static final double VELOCITY_CONVERSION_FACTOR = Units.inchesToMeters(4) * Math.PI / 6.12;
        private static final double POSITION_CONVERSION_FACTOR = Units.inchesToMeters(4) * Math.PI / 6.12;
        private static final double ROTATION_VELOCITY_CONVERSION_FACTOR = (2 * Math.PI) / (21.4285714);

        public static final PIDConstants TRANSLATION_PID_PP_CONSTANTS = new PIDConstants(10.0, 0.0, 0.0); //TODO
        public static final PIDConstants ANGLE_PID_PP_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
        public static final Gains ANGLE_PID_GAINS = new Gains(2.0, 0.0, 0.0); //2.5, 0.0, 1.0);
        public static final Gains TRANSLATION_PID_GAINS = new Gains(15.0, 0.0, 0.0);

        private static final IMU GYRO = new Pigeon(GYRO_ID, SWERVE_CANBUS, new Rotation3d());

        public static final double DEADBAND_VALUE = 0.15;

        public static Swerve configureSwerve(Pose2d initialPose) {
            return new Swerve(
                    new ModulesHolder(
                            new SwerveModule(
                                    new TalonFXMotor(FRONT_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                    new SparkMaxMotor(FRONT_LEFT_ROTATION_ID, kBrushless),
                                    new Gains(6, 0, 0, 0.32621, 2.6212, 0.33422, 0),
                                    new Gains(0, 0, 0, 0.0092262, 2.22144, 0.61204, 0), //TODO
                                    PID_TOLERANCE,
                                    FRONT_LEFT_TRANSLATION,
                                    () -> FRONT_LEFT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(FRONT_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                                    new SparkMaxMotor(FRONT_RIGHT_ROTATION_ID, kBrushless),
                                    new Gains(6, 0, 0, 0.32621, 2.6212, 0.33422, 0),
                                    new Gains(0, 0, 0, 0.0092262, 2.22144, 0.61204, 0), //TODO
                                    PID_TOLERANCE,
                                    FRONT_RIGHT_TRANSLATION,
                                    () -> FRONT_RIGHT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(BACK_LEFT_DRIVE_ID, SWERVE_CANBUS),
                                    new SparkMaxMotor(BACK_LEFT_ROTATION_ID, kBrushless),
                                    new Gains(7, 0, 0, 0.32621, 2.6212, 0.33422, 0),
                                    new Gains(0, 0, 0, 0.0092262, 2.22144, 0.61204, 0), //TODO
                                    PID_TOLERANCE,
                                    BACK_LEFT_TRANSLATION,
                                    () -> BACK_LEFT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            ),
                            new SwerveModule(
                                    new TalonFXMotor(BACK_RIGHT_DRIVE_ID, SWERVE_CANBUS),
                                    new SparkMaxMotor(BACK_RIGHT_ROTATION_ID, kBrushless),
                                    new Gains(6, 0, 0, 0.32621, 2.6212, 0.33422, 0),
                                    new Gains(0, 0, 0, 0.0092262, 2.22144, 0.61204, 0), //TODO
                                    PID_TOLERANCE,
                                    BACK_RIGHT_TRANSLATION,
                                    () -> BACK_RIGHT_ABS_ENCODER.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI,
                                    MAX_MODULE_VEL,
                                    VELOCITY_CONVERSION_FACTOR,
                                    POSITION_CONVERSION_FACTOR,
                                    ROTATION_VELOCITY_CONVERSION_FACTOR
                            )),
                    GYRO,
                    initialPose
            );
        }
    }

    public static class FieldConstants {
        public static final Translation2d REEF_CENTER = new Translation2d(4.4893484, 4.026);
        private static final Translation2d B1 = new Translation2d(5.7668696, 3.86156696), B12 = new Translation2d(5.7668696, 4.19023304);
        private static final Translation2d BASE_L1 = new Translation2d(5.7668696, 4.0259);
        private static final Translation2d BASE_ALGAE = new Translation2d(5.7668696 + 1, 4.0259); //TODO: find x
        private static final Translation2d BASE_POST_ALGAE = new Translation2d(5.7668696 + 1, 4.0259); //TODO: find x

        public static final AlliancePose[] LEFT_BRANCHES = {
                new AlliancePose(B1, Rotation2d.fromDegrees(180)),
                new AlliancePose(B1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120)),
                new AlliancePose(B1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(60)),
                new AlliancePose(B1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d()),
                new AlliancePose(B1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(B1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120)),
        };
        public static final AlliancePose[] RIGHT_BRANCHES = {
                new AlliancePose(B12, Rotation2d.fromDegrees(180)),
                new AlliancePose(B12.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120)),
                new AlliancePose(B12.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(60)),
                new AlliancePose(B12.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d()),
                new AlliancePose(B12.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(B12.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120))
        };

        public static final AlliancePose[] L1s = {
                new AlliancePose(BASE_L1, Rotation2d.fromDegrees(180)),
                new AlliancePose(BASE_L1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120)),
                new AlliancePose(BASE_L1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(60)),
                new AlliancePose(BASE_L1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-180)), Rotation2d.fromDegrees(0)),
                new AlliancePose(BASE_L1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(BASE_L1.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120)),
        };

//        public static final Pose2d[] ALGAES = {
//                new Pose2d(BASE_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(180)),
//                new Pose2d(BASE_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(120)),
//                new Pose2d(BASE_ALGAE, Rotation2d.fromDegrees(60)),
//                new Pose2d(BASE_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(0)),
//                new Pose2d(BASE_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(-60)),
//                new Pose2d(BASE_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-180)), Rotation2d.fromDegrees(-120)),
//        };
//        public static final Pose2d[] POST_ALGAES = {
//                new Pose2d(BASE_POST_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(180)),
//                new Pose2d(BASE_POST_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(120)),
//                new Pose2d(BASE_POST_ALGAE, Rotation2d.fromDegrees(60)),
//                new Pose2d(BASE_POST_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(0)),
//                new Pose2d(BASE_POST_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(-60)),
//                new Pose2d(BASE_POST_ALGAE.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-120)),
//        };
//
//        public static final Pose2d[] FEADERS_POSES = {
//                new Pose2d(),
//                new Pose2d(),
//                new Pose2d(),
//                new Pose2d(),
//                new Pose2d(),
//                new Pose2d()
//        };
//
//        public static final Pose2d[] NET_POSES = {
//                new Pose2d(),
//                new Pose2d(),
//                new Pose2d()
//        };
//
    }

    public static final int PROCESSOR_ID = 1;
    public static final int NET_ID = 4;
}
