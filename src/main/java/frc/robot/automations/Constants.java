package frc.robot.automations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.excalib.additional_utilities.AllianceUtils;

import static frc.excalib.additional_utilities.AllianceUtils.*;

public final class Constants {
    public static class FieldConstants {
        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.4893484, 4.026);
        public static final Translation2d RED_REEF_CENTER = new Translation2d(FIELD_LENGTH_METERS - BLUE_REEF_CENTER.getX(), BLUE_REEF_CENTER.getY());

        public static final double ANGLE_TOLERANCE = 0.1;
        public static final double x_TOLERANCE = 0.1;
        public static final double Y_TOLERANCE = 0.1;

        public static Translation2d getReefCenter() {
            if (AllianceUtils.isBlueAlliance()) {
                return BLUE_REEF_CENTER;
            }
            return RED_REEF_CENTER;
        }

        private static final Translation2d B1 = new Translation2d(5.7668696, 3.812+0.02), B12 = new Translation2d(5.7668696, 4.142+0.02);
        private static final Translation2d BASE_L1 = new Translation2d(5.7668696, 4.0259);
        private static final Translation2d BASE_ALGAE = new Translation2d(5.7668696, 4.0259-0.02); //TODO: find x
        private static final Translation2d BASE_POST_ALGAE = new Translation2d(6.3, 4.0259-0.02); //TODO: find x
        private static final Translation2d BASE_GENERAL = new Translation2d(6.3, 4.0259);
        public static final double NET_X_VALUE = 7.721;
        public static final double POST_NET_X_VALUE = 7.418;

        public static AlliancePose[] GENERAL_POSES = {
                new AlliancePose(BASE_GENERAL, Rotation2d.fromDegrees(180)),
                new AlliancePose(BASE_GENERAL.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120)),
                new AlliancePose(BASE_GENERAL.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(60)),
                new AlliancePose(BASE_GENERAL.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d()),
                new AlliancePose(BASE_GENERAL.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(BASE_GENERAL.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120)),
        };

        public static final AlliancePose[] LEFT_BRANCHES = {
                new AlliancePose(B1, Rotation2d.fromDegrees(180)),
                new AlliancePose(B1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120)),
                new AlliancePose(B1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(60)),
                new AlliancePose(B1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d()),
                new AlliancePose(B1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(B1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120)),
        };

        public static final AlliancePose[] RIGHT_BRANCHES = {
                new AlliancePose(B12, Rotation2d.fromDegrees(180)),
                new AlliancePose(B12.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(120)),
                new AlliancePose(B12.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(60)),
                new AlliancePose(B12.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-180)), new Rotation2d()),
                new AlliancePose(B12.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(B12.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120))
        };

        public static final AlliancePose[] L1s = {
                new AlliancePose(BASE_L1, Rotation2d.fromDegrees(0)),
                new AlliancePose(BASE_L1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(BASE_L1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(-120)),
                new AlliancePose(BASE_L1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-180)), Rotation2d.fromDegrees(180)),
                new AlliancePose(BASE_L1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(120)),
                new AlliancePose(BASE_L1.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(60)),
        };

        public static final AlliancePose[] ALGAES = {
                new AlliancePose(BASE_ALGAE, Rotation2d.fromDegrees(180)),
                new AlliancePose(BASE_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-240)),
                new AlliancePose(BASE_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(-300)),
                new AlliancePose(BASE_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-180)), Rotation2d.fromDegrees(0)),
                new AlliancePose(BASE_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(BASE_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120)),
        };

        public static final AlliancePose[] POST_ALGAES = {
                new AlliancePose(BASE_POST_ALGAE, Rotation2d.fromDegrees(180)),
                new AlliancePose(BASE_POST_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-60)), Rotation2d.fromDegrees(-240)),
                new AlliancePose(BASE_POST_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-120)), Rotation2d.fromDegrees(-300)),
                new AlliancePose(BASE_POST_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-180)), Rotation2d.fromDegrees(-180)),
                new AlliancePose(BASE_POST_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-240)), Rotation2d.fromDegrees(-60)),
                new AlliancePose(BASE_POST_ALGAE.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-300)), Rotation2d.fromDegrees(-120)),
        };

        public static final AlliancePose[] FEEDERS_POSES = {
                new AlliancePose(new Translation2d(0.91, FIELD_WIDTH_METERS - 6.96), new Rotation2d(Units.degreesToRadians(54))),
                new AlliancePose(new Translation2d(1.12, FIELD_WIDTH_METERS - 7.02), new Rotation2d(Units.degreesToRadians(54))),
                new AlliancePose(new Translation2d(1.25, FIELD_WIDTH_METERS - 7.19), new Rotation2d(Units.degreesToRadians(54))),
                new AlliancePose(new Translation2d(0.91, 6.96), new Rotation2d(Units.degreesToRadians(306))),
                new AlliancePose(new Translation2d(1.12, 7.02), new Rotation2d(Units.degreesToRadians(306))),
                new AlliancePose(new Translation2d(1.25, 7.19), new Rotation2d(Units.degreesToRadians(306)))
        };

        public static final AlliancePose[] NET_POSES = {
                new AlliancePose(NET_X_VALUE, 7.5, 0),
                new AlliancePose(NET_X_VALUE, 7.0, 0),
                new AlliancePose(NET_X_VALUE, 6.5, 0),
                new AlliancePose(NET_X_VALUE, 5, 0),
                new AlliancePose(NET_X_VALUE, 5.5, 0),
                new AlliancePose(NET_X_VALUE, 5, 0)
        };

        public static final AlliancePose[] POST_NET_POSES = {
                new AlliancePose(POST_NET_X_VALUE, 7.5, 0),
                new AlliancePose(POST_NET_X_VALUE, 7.0, 0),
                new AlliancePose(POST_NET_X_VALUE, 6.5, 0),
                new AlliancePose(POST_NET_X_VALUE, 5, 0),
                new AlliancePose(POST_NET_X_VALUE, 5.5, 0),
                new AlliancePose(POST_NET_X_VALUE, 5, 0)
        };
    }
}
