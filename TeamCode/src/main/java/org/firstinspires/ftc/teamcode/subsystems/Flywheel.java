package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Flywheel {
    public DcMotorEx rightMotor;
    public DcMotorEx leftMotor;
    
    // RPM targets
    private double targetRPM = 0;
    private double currentTargetRPM = 0; // ramp limited
    
    // Constants
    public static final int TICKS_PER_REV = 28;
    public static final double MAX_ACCEL_RPM_PER_SEC = 3000;
    
    public double filteredVoltage = 13;
    
    public static double alpha = 0.01;
    public static double kP = 0.005;
    public static double kI = 0.002;
    public static double kS = 0.85;
    public static double kV = 0.0022;
    public static double I_ENABLE_ERROR = 300;
    
    private double totalError = 0;
    private boolean shotDetected;
    private double lastRPM;
    
    // http://192.168.43.1:8080/dash
    
    public Flywheel(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void setRPM(double rpm) {
        targetRPM = rpm;
    }
    
    public void stop() {
        targetRPM = 0;
        totalError = 0;
    }
    
    public boolean rpmUnderTarget() {
        return getRpm() < getTargetRPM() * 0.80;
    }
    
    public double getRpm() {
        return ticksPerSecondToRpm(rightMotor.getVelocity());
    }
    
    public double getTargetRPM() {
        return targetRPM;
    }
    
    public static double minRPM = 2064.38;
 
    public void setRpmFromDistance(double dist) {
       // setRPM(13.5 * dist + 2373);
        setRPM(17.17 * dist + minRPM);
        
        // y=17.17072x+2064.38333
    }
    
    public boolean artifactLaunched() {
        return shotDetected;
    }
    
    double shotCooldown = 0.15;   // seconds
    double lastShotTime = System.nanoTime();

    boolean updateShotDetection(double rpm, double now) {
        double delta = rpm - lastRPM;
        lastRPM = rpm;

        // Not ready to detect yet
        if (now - lastShotTime < shotCooldown) return false;

        // Detect sharp RPM drop
        if (delta < -300) {            // tuned threshold
            lastShotTime = now;
            return true;
        }

        return false;
    }
    
    public void update(double dt, double voltage) {
        double currentRpm = getRpm();
        
        shotDetected = updateShotDetection(targetRPM, System.nanoTime());

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
        
        // ---- PID ----
        double error = currentTargetRPM - currentRpm;
        double p = kP * error;
        
        // Conditional integral
        if (Math.abs(error) < I_ENABLE_ERROR) {
            totalError += error * dt;
            filteredVoltage = filteredVoltage + alpha * (voltage - filteredVoltage);
        }
        double i = kI * totalError;
        
        double power = (ff + p + i) / filteredVoltage;
        
        power = Range.clip(power, 0, 1);
        
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    
    private double ticksPerSecondToRpm(double tps) {
        return tps / TICKS_PER_REV * 60.0;
    }
    
    public boolean isUpToSpeed() {
        if (targetRPM < 100) return false;
        
        double rpmError = Math.abs(getRpm() - targetRPM);
        
        return rpmError < 50;
    }
}
