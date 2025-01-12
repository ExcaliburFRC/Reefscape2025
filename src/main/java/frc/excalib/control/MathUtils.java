package frc.excalib.control;

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
}
