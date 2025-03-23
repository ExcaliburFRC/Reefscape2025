package frc.excalib.control.math;

import edu.wpi.first.math.geometry.Translation2d;

public class Circle {
    public final double r;
    public final Translation2d center;

    public Circle(double a, double b, double r) {
        this.center = new Translation2d(a, b);
        this.r = r;
    }

    public Line getTangent(Translation2d pointOnCircle) {
        return new Line(
                pointOnCircle.getX() - center.getX(),
                pointOnCircle.getY() - center.getY(),
                -center.getX() * (pointOnCircle.getX() - center.getX())
                        - center.getY() * (pointOnCircle.getY() - center.getY()) -
                        this.r * this.r
        );
    }

    public Line[] getTangents(Translation2d point) {
        if (point.getDistance(this.center) < this.r) return new Line[0];
        else if (point.getDistance(this.center) == this.r) {
            return new Line[]{
                    getTangent(point)
            };
        }
        double centersDistance = this.center.getDistance(point);
        double newRad = Math.sqrt(Math.pow(this.r, 2) - Math.pow(centersDistance, 2));
        Circle newCircle = new Circle(point.getX(), point.getY(), newRad);
        Translation2d[] intersections = getInterSections(newCircle);
        Translation2d firstTanPoint = intersections[0];
        Translation2d secondTanPoint = intersections[1];

        return new Line[]{
                getTangent(firstTanPoint),
                getTangent(secondTanPoint)
        };
    }

    public Translation2d[] getInterSections(Circle other) {
        double d = this.center.getDistance(other.center);
        if (d > this.r + other.r || d < Math.abs(this.r - other.r)) {
            return new Translation2d[0]; // No intersection
        }

        double a = (this.r * this.r - other.r * other.r + d * d) / (2 * d);
        double h = Math.sqrt(this.r * this.r - a * a);

        double x0 = this.center.getX() + a * (other.center.getX() - this.center.getX()) / d;
        double y0 = this.center.getY() + a * (other.center.getY() - this.center.getY()) / d;

        double rx = -(other.center.getY() - this.center.getY()) * (h / d);
        double ry = (other.center.getX() - this.center.getX()) * (h / d);

        Translation2d p1 = new Translation2d(x0 + rx, y0 + ry);
        Translation2d p2 = new Translation2d(x0 - rx, y0 - ry);

        if (d == this.r + other.r || d == Math.abs(this.r - other.r)) {
            return new Translation2d[]{p1}; // One intersection (tangential circles)
        }
        return new Translation2d[]{p1, p2}; // Two intersections
    }
}
