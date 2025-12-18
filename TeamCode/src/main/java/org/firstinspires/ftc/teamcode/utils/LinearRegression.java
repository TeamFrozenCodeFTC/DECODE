package org.firstinspires.ftc.teamcode.utils;

import java.util.function.DoubleUnaryOperator;

public class LinearRegression {
    public static DoubleUnaryOperator fit(double[][] points) {
        if (points == null || points.length == 0) {
            throw new IllegalArgumentException("Points array is empty");
        }

        int n = points.length;

        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumXX = 0;

        for (double[] p : points) {
            double x = p[0];
            double y = p[1];

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
        }

        double slope =
                (n * sumXY - sumX * sumY) /
                (n * sumXX - sumX * sumX);

        double intercept = (sumY - slope * sumX) / n;

        return x -> slope * x + intercept;
    }
}
