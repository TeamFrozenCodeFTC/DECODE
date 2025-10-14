package org.firstinspires.ftc.blackice.core.paths.geometry;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

import java.util.Map;
import java.util.TreeMap;

/**
 * This is the BezierCurve class. This class handles the creation of Bezier curves, which are used
 * as the basis of the path for the Path class. Bezier curves are parametric curves defined by a set
 * of control points. So, they take in one input, t, that ranges from [0, 1] and that returns a point
 * on the curve. You can read more on Bezier curves here:
 * <a href="https://en.wikipedia.org/wiki/B">Bézier Curve</a>
 */
public class BezierCurve implements PathGeometry {
    private final Vector[] controlPoints;
    private final double length;
    private final PathPoint endPathPoint;
    private final TreeMap<Double, Double> arcLengthLookup = new TreeMap<>();

    public BezierCurve(Vector... controlPoints) {
        this.controlPoints = controlPoints;
        this.length = calculateAndCacheArcLength(1000);
        this.endPathPoint = new PathPoint(this.controlPoints[this.controlPoints.length - 1],
                                          calculateFirstDerivative(1), 0, this.length, 0, 1, 1);
    }
    
    @Override
    public BezierCurve reversed() {
        Vector[] reversedControlPoints = new Vector[controlPoints.length];
        for (int i = 0; i < controlPoints.length; i++) {
            reversedControlPoints[i] = controlPoints[controlPoints.length - 1 - i];
        }
        return new BezierCurve(reversedControlPoints);
    }
    
    @Override
    public BezierCurve mirrored() {
        Vector[] mirroredControlPoints = new Vector[controlPoints.length];
        for (int i = 0; i < controlPoints.length; i++) {
            mirroredControlPoints[i] = controlPoints[i].mirroredAcrossYAxis();
        }
        return new BezierCurve(mirroredControlPoints);
    }
    
    @Override
    public double length() {
        return length;
    }
    
    @Override
    public PathPoint getEndPathPoint() {
        return endPathPoint;
    }
    
    /**
     * Computes the scalar curvature of a 2D parametric curve
     * using its first and second derivatives.
     */
    public static double computeCurvature(Vector firstDerivative, Vector secondDerivative) {
        double dx = firstDerivative.getX();
        double dy = firstDerivative.getY();
        double ddx = secondDerivative.getX();
        double ddy = secondDerivative.getY();
        
        double numerator = dx * ddy - dy * ddx;
        double denominator = Math.pow(dx * dx + dy * dy, 1.5);
        
        if (denominator <= 1e-6) return 0.0;
        
        return numerator / denominator;
    }
    
    
    @Override
    public PathPoint computeClosestPathPointTo(Vector target, double startingGuess) {
        double t = startingGuess;
        int maxIterations = 100;
        int iteration = 0;

        Vector firstDerivative;
        Vector secondDerivative;
        Vector bezierPoint;

        do {
            bezierPoint = computePointAt(t);
            firstDerivative = calculateFirstDerivative(t);

            Vector errorVector = target.minus(bezierPoint);

            secondDerivative = calculateSecondDerivative(t);

            double denominator = firstDerivative.lengthSquared() + errorVector.dotProduct(secondDerivative);
            
            if (Math.abs(denominator) < 1e-6) break; // Prevent divide-by-near-zero
            
            double numerator = errorVector.dotProduct(firstDerivative);

            double deltaT = numerator / denominator;
            
            t = Range.clip(t + deltaT, 0, 1);
            if (Math.abs(deltaT) < 1e-6) {
                break;
            }

            iteration++;
        } while (iteration <= maxIterations);

        Vector tangent = firstDerivative.normalized();
        double curvature = computeCurvature(firstDerivative, secondDerivative);
        double arcLength = getArcLengthAt(t);
        return new PathPoint(bezierPoint, tangent, curvature, arcLength,
                             length - arcLength,
                             arcLength / length, t);
    }
    
    public double getArcLengthAt(double t) {
        if (t <= 0) return 0;
        if (t >= 1) return length;
        
        Map.Entry<Double, Double> floor = arcLengthLookup.floorEntry(t);
        Map.Entry<Double, Double> ceil = arcLengthLookup.ceilingEntry(t);
        
        if (floor == null || ceil == null || floor.getKey().equals(ceil.getKey())) {
            return floor != null ? floor.getValue() : 0;
        }
        
        double t1 = floor.getKey();
        double s1 = floor.getValue();
        double t2 = ceil.getKey();
        double s2 = ceil.getValue();
        
        double ratio = (t - t1) / (t2 - t1);
        return s1 + ratio * (s2 - s1);
    }
    
    public Vector computePointAt(double t) {
        int n = controlPoints.length - 1;
        Vector point = new Vector(0, 0);
        
        for (int i = 0; i <= n; i++) {
            double bernstein = binomial(n, i) * Math.pow(1 - t, n - i) * Math.pow(t, i);
            point = point.plus(controlPoints[i].times(bernstein));
        }
        
        return point;
    }
    
    /**
     * This is equal to the tangent vector of the curve at t.
     */
    public Vector calculateFirstDerivative(double t) {
        int n = controlPoints.length - 1;
        Vector derivative = new Vector(0, 0);
        
        for (int i = 0; i < n; i++) {
            double bernstein = binomial(n - 1, i) * Math.pow(1 - t, n - 1 - i) * Math.pow(t, i);
            Vector diff = controlPoints[i + 1].minus(controlPoints[i]);
            derivative = derivative.plus(diff.times(bernstein));
        }
        
        return derivative.times(n);
    }
    
    public Vector calculateSecondDerivative(double t) {
        int n = controlPoints.length - 1;
        Vector secondDerivative = new Vector(0, 0);
        
        for (int i = 0; i <= n - 2; i++) {
            double bernstein = binomial(n - 2, i) * Math.pow(1 - t, n - 2 - i) * Math.pow(t, i);
            Vector diff = controlPoints[i + 2]
                .minus(controlPoints[i + 1].times(2))
                .plus(controlPoints[i]);
            secondDerivative = secondDerivative.plus(diff.times(bernstein));
        }
        
        return secondDerivative.times(n * (n - 1));
    }
    
    private double calculateAndCacheArcLength(int samples) {
        arcLengthLookup.clear();
        double length = 0;
        Vector prev = computePointAt(0);
        arcLengthLookup.put(0.0, 0.0);
        
        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            Vector point = computePointAt(t);
            double segment = point.minus(prev).computeMagnitude();
            length += segment;
            arcLengthLookup.put(t, length);
            prev = point;
        }
        
        return length;
    }
    
    private int binomial(int n, int k) {
        int result = 1;
        for (int i = 1; i <= k; i++) {
            result *= (n - (k - i));
            result /= i;
        }
        return result;
    }
}
