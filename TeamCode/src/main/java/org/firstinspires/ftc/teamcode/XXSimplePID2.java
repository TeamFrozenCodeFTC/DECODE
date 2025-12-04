package org.firstinspires.ftc.teamcode;

public class XXSimplePID2 {
    // PID gains (stable defaults for FTC flywheels)
    private double kP = 0.0009;
    private double kI = 0.00012;
    private double kD = 0.00015;

    // Feed-forward tuned for Yellowjacket 6000 RPM motor
    private double kS = 0.06;      // static friction feed-forward
    private double kV = 0.0020;    // voltage per RPM (approx 1/5000)

    // State
    private double setpointRPM = 0;
    private double integral = 0;
    private double lastMeasurement = 0;
    private boolean first = true;

    private double iMin = -4000;
    private double iMax = 4000;

    public void setSetpointRPM(double rpm) {
        setpointRPM = rpm;
    }

    public double calculate(double measuredRPM, double dt) {
        double error = setpointRPM - measuredRPM;

        // ----- P -----
        double p = kP * error;

        // ----- I -----
        integral += error * dt;
        if (integral > iMax) integral = iMax;
        if (integral < iMin) integral = iMin;
        double i = kI * integral;

        // ----- D (on measurement for noise reduction) -----
        double derivative = 0;
        if (!first) {
            derivative = (lastMeasurement - measuredRPM) / dt;
        }
        double d = kD * derivative;

        // ----- Feed Forward -----
        double f =
                kS * Math.signum(setpointRPM) +
                        kV * setpointRPM;

        // Save state
        lastMeasurement = measuredRPM;
        first = false;

        // Motor output (FTC range -1 to 1)
        double output = p + i + d + f;
        return clamp(output, -1, 1);
    }

    public void reset() {
        integral = 0;
        first = true;
    }

    private double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(max, x));
    }
}
