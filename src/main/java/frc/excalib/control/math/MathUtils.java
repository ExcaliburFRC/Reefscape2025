package frc.excalib.control.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.awt.geom.Line2D;

public class MathUtils {
    /**
     * A function that checks the minimum value in abs
     *
     * @param sizeLimit max size
     * @return minimum double value
     */

    public static double minSize(double val, double sizeLimit) {
        return Math.min(sizeLimit, Math.abs(val)) * Math.signum(val);
    }

    public static double limitTo(double limit, double value) {
        if ((limit > 0 && limit < value) || (limit < 0 && limit > value)) {
            return limit;
        }
        return value;
    }

    public Translation2d getTargetPose(Translation2d robot, Translation2d target, Translation2d reefCenter) {
        double radius = reefCenter.getDistance(target);
        Circle c = new Circle(reefCenter.getX(), reefCenter.getY(), radius);
        Line[] tangents = c.getTangents(robot);
        if (tangents.length == 0) return new Translation2d(radius, robot.minus(reefCenter).getAngle());
        Line targetTangent = c.getTangent(target);
        if (tangents.length == 1) {
            return targetTangent.findIntersection(tangents[0]);
        }
        Translation2d translation1 = targetTangent.findIntersection(tangents[0]);
        Translation2d translation2 = targetTangent.findIntersection(tangents[1]);
        if (robot.getDistance(translation1) < robot.getDistance(translation2)) return translation1;
        return translation2;
    }
}
