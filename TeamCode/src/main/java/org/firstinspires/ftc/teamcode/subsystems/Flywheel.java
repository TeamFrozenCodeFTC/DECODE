package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.LinearRegression;

import java.util.function.DoubleUnaryOperator;

@Config // http://192.168.43.1:8080/dash
public class Flywheel {
    public DcMotorEx rightMotor;
    public DcMotorEx leftMotor;

    private double targetRPM = 0;
    private double currentTargetRPM = 0;

    public static final int TICKS_PER_REV = 28;
//    public static final double MAX_ACCEL_RPM_PER_SEC = 3000;
    public static final double MAX_ACCEL_RPM_PER_SEC = 6000;
    
    public double filteredVoltage = 13;
    
    // Tunable Constants
    public static double alpha = 0.1;
//    public static double kP = 0.005;
//    public static double kI = 0.002;
//    public static double kS = 0.85;
//    public static double kV = 0.0022;
    public static double kP = 0.01;
    public static double kI = 0.003;
    public static double kS = 0.5;
    public static double kV = 0.0022;
    public static double shotCooldown = 0.15;
    public static double I_ENABLE_ERROR = 300;
    public static double RPM_TOLERANCE = 50;
    
    private double lastShotTime = 0;
    private double totalError = 0;
    private double lastRPM = 0;
    private boolean shotDetected = false;
    
    public double manualAdjustmentMultiplier = 1;
    // 0.00225, 0.5
    private final DoubleUnaryOperator distanceToRpm =
        LinearRegression.fit(new double[][]{
            //{54.48908, 3000}, // y=17.17072x+2064.38333

//            {95.6, 3700},
//            {135.3, 4500}
            {141, 4450},
            {100.12, 3850}
        });
    
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
    
    public double getTargetRPM() {
        return targetRPM;
    }
    
    public void setRpmFromDistance(double dist) {
        setRPM(distanceToRpm.applyAsDouble(dist) * manualAdjustmentMultiplier);
    }
    
    public boolean artifactLaunched() {
        return shotDetected;
    }

    boolean updateShotDetection(double rpm) {
        double delta = rpm - lastRPM;
        lastRPM = rpm;
        
        double now = System.currentTimeMillis() / 1000.0;

        if (now - lastShotTime < shotCooldown) return false;
      
        if (delta < -300) {
            lastShotTime = now;
            return true;
        }

        return false;
    }
    
    public void update(double dt, double voltage) {
        double currentRpm = getRpm();
        
        shotDetected = updateShotDetection(currentRpm);

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
    
    public double getRpm() {
        return ticksPerSecondToRpm(rightMotor.getVelocity());
    }
    
    private double ticksPerSecondToRpm(double tps) {
        return tps / TICKS_PER_REV * 60.0;
    }
    
    public boolean isUpToSpeed() {
        if (targetRPM < RPM_TOLERANCE) return false;
        
        double rpmError = Math.abs(getRpm() - targetRPM);
        
        return rpmError < RPM_TOLERANCE;
    }
}
