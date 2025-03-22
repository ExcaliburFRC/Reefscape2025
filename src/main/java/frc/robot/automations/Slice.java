package frc.robot.automations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.excalib.additional_utilities.AllianceUtils;

import static frc.excalib.additional_utilities.AllianceUtils.*;
import static frc.robot.automations.Constants.*;
import static frc.robot.automations.Constants.FieldConstants.*;

public enum Slice {
    FIRST(GENERAL_POSES[0], LEFT_BRANCHES[0], RIGHT_BRANCHES[0], L1s[0], ALGAES[0], 2),
    SECOND(GENERAL_POSES[1], LEFT_BRANCHES[1], RIGHT_BRANCHES[1], L1s[1], ALGAES[1], 3),
    THIRD(GENERAL_POSES[2], LEFT_BRANCHES[2], RIGHT_BRANCHES[2], L1s[2], ALGAES[2], 2),
    FOURTH(GENERAL_POSES[3], LEFT_BRANCHES[3], RIGHT_BRANCHES[3], L1s[3], ALGAES[3], 3),
    FIFTH(GENERAL_POSES[4], LEFT_BRANCHES[4], RIGHT_BRANCHES[4], L1s[4], ALGAES[4], 2),
    SIXTH(GENERAL_POSES[5], LEFT_BRANCHES[5], RIGHT_BRANCHES[5], L1s[5], ALGAES[5], 3);

    public final AlliancePose generalPose, leftPose, rightPose, l1Pose, alagePose;
    public final int ALGAE_LEVEL;

    Slice(AlliancePose generalPose, AlliancePose leftPose, AlliancePose rightPose, AlliancePose l1Pose, AlliancePose alagePose, int ALGAE_LEVEL) {
        this.generalPose = generalPose;
        this.leftPose = leftPose;
        this.rightPose = rightPose;
        this.l1Pose = l1Pose;
        this.alagePose = alagePose;
        this.ALGAE_LEVEL = ALGAE_LEVEL;
    }

    public static Slice getSlice(Translation2d translation) {
        Translation2d robotTranslation = translation;
        if (AllianceUtils.isRedAlliance()) {
            robotTranslation = new Translation2d(FIELD_LENGTH_METERS - robotTranslation.getX(), FIELD_WIDTH_METERS - robotTranslation.getY());
        }
        robotTranslation = robotTranslation.minus(BLUE_REEF_CENTER);
        double angle = robotTranslation.getAngle().getDegrees();
        if (angle < 30 && angle > -30) {
            return FIRST;
        }
        if (angle < -30 && angle > -90) {
            return SECOND;
        }
        if (angle < -90 && angle > -150) {
            return THIRD;
        }
        if (angle > 150 || angle < -150) {
            return FOURTH;
        }
        if (angle > 90 && angle < 150) {
            return FIFTH;
        }
        return SIXTH;
    }

    public static AlliancePose getBranchPose(boolean right, Translation2d translation){
        return right? getSlice(translation).rightPose : getSlice(translation).leftPose;
    }
}
