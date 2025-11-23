package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Flywheel2 {
    public DcMotorEx rightMotor;
    public DcMotorEx leftMotor;
    
    // RPM targets
    private double targetRPM = 0;
    private double currentTargetRPM = 0; // ramp limited
    
    // Constants
    public static final int TICKS_PER_REV = 28;
    public static final double MAX_ACCEL_RPM_PER_SEC = 3000;
    
    // PID + Feedforward
    public static double kP = 0.0006;
    public static double kI = 0.0;
    public static double kS = 0.02;
    public static double kV = 0.000185;
    public static double I_ENABLE_ERROR = 300;

    // State
    private double lastRpm = 0;
    private double totalError = 0;
    
    // http://192.168.43.1:8080/dash
    
    // Acceleration window for "isUpToSpeed"
    private static final int ACCEL_SAMPLES = 10;
    private final double[] accelWindow = new double[ACCEL_SAMPLES];
    private int accelIndex = 0;
    private boolean accelFilled = false;
    
    public Flywheel2(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    double comp = 1;
    public void updateVoltComp(double comp) {
        this.comp = comp;
    }
    
    public void setRPM(double rpm) {
        targetRPM = rpm;
    }
    
    public void stop() {
        targetRPM = 0;
    }
    
    public double getRpm() {
        return ticksPerSecondToRpm(
            (rightMotor.getVelocity() + leftMotor.getVelocity()) / 2.0
        );
    }
    
    public double getTargetRPM() {
        return targetRPM;
    }
 
    public void setRpmFromDistance(double dist) {
        setRPM(13.5 * dist + 2373);
    }
    
    // -------------------------
    // Main loop
    // Call every OpMode iteration
    // -------------------------
    
    public void update(double dt) {
        double currentRpm = getRpm();
        
        // ---- acceleration tracking ----
        double accel = (currentRpm - lastRpm) / dt;
        accelWindow[accelIndex++] = accel;
        if (accelIndex >= ACCEL_SAMPLES) {
            accelIndex = 0;
            accelFilled = true;
        }
        lastRpm = currentRpm;
        
        // ---- ramp target RPM for stability ----
        double diff = targetRPM - currentTargetRPM;
        double maxStep = MAX_ACCEL_RPM_PER_SEC * dt;
        
        if (Math.abs(diff) > maxStep)
            currentTargetRPM += Math.copySign(maxStep, diff);
        else
            currentTargetRPM = targetRPM;
        
        // ---- Feedforward ----
        double ff = kS * Math.signum(currentTargetRPM)
            + kV * currentTargetRPM;
        
        // Voltage compensation
        ff *= comp;
        
        // ---- PID ----
        double error = currentTargetRPM - currentRpm;
        double p = kP * error;
        
        // Conditional integral
        if (Math.abs(error) < I_ENABLE_ERROR) {
            totalError += error * dt;
        }
        double i = kI * totalError;
        
        double power = ff + p + i;
        
        power = Range.clip(power, 0, 1);
        
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    
    private double ticksPerSecondToRpm(double tps) {
        return tps / TICKS_PER_REV * 60.0;
    }
    
    private double getAverageAccel() {
        int n = accelFilled ? ACCEL_SAMPLES : accelIndex;
        double sum = 0;
        for (int i = 0; i < n; i++) sum += accelWindow[i];
        return n == 0 ? 99999 : sum / n;
    }
    
    public boolean isUpToSpeed() {
        if (targetRPM < 100) return false;
        
        double rpmError = Math.abs(getRpm() - targetRPM);
        double accel = Math.abs(getAverageAccel());
        
        return rpmError < 50 && accel < 100;
    }
}
