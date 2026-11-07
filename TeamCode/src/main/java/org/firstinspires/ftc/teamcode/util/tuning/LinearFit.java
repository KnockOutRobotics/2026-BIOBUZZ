package org.firstinspires.ftc.teamcode.util.tuning;

/** Least-squares line through (x, y) points: y = slope * x + intercept. */
public class LinearFit {

    private int n;
    private double sumX;
    private double sumY;
    private double sumXX;
    private double sumXY;
    private double sumYY;

    public void add(double x, double y) {
        n++;
        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
        sumYY += y * y;
    }

    public void clear() {
        n = 0;
        sumX = sumY = sumXX = sumXY = sumYY = 0;
    }

    public int count() {
        return n;
    }

    /** NaN until there are two points with different x. */
    public double slope() {
        double denominator = n * sumXX - sumX * sumX;
        if (n < 2 || Math.abs(denominator) < 1e-12) {
            return Double.NaN;
        }
        return (n * sumXY - sumX * sumY) / denominator;
    }

    public double intercept() {
        double slope = slope();
        return Double.isNaN(slope) ? Double.NaN : (sumY - slope * sumX) / n;
    }

    /** 1 = the points lie on the line; NaN with fewer than two distinct points. */
    public double rSquared() {
        double xVariance = n * sumXX - sumX * sumX;
        double yVariance = n * sumYY - sumY * sumY;
        if (n < 2 || Math.abs(xVariance) < 1e-12) {
            return Double.NaN;
        }
        if (Math.abs(yVariance) < 1e-12) {
            return 1; // every y equal and the line is flat through them
        }
        double covariance = n * sumXY - sumX * sumY;
        return covariance * covariance / (xVariance * yVariance);
    }
}
