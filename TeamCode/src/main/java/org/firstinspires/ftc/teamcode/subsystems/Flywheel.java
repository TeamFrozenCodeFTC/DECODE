package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.LinearRegression;

import java.util.function.DoubleUnaryOperator;

@Config
public class Flywheel {
    public static final int TICKS_PER_REV = 28;
    public static final double MAX_ACCEL_RPM_PER_SEC = 4000;
    
    public static double kP = 0.002;
    public static double kI = 0;
    public static double kS = 0.6;
    public static double kV = 0.00225;
    public static double I_ENABLE_ERROR = 300;
    public static double RPM_TOLERANCE = 50;
    
    public static double REV_DELTA_TIME = 0.05;
    public static double RPM_RESOLUTION
        = 60.0 / (TICKS_PER_REV * REV_DELTA_TIME);
    
    public DcMotorEx rightMotor;
    public DcMotorEx leftMotor;
    
    public enum State {
        OFF,
        SPINNING_UP,
        AT_SPEED
    }
    
    private State state = State.OFF;
    
    private double targetRPM = 0;
    private double currentTargetRPM = 0;
    private double totalError = 0;
    private double currentError = 0;
    private double currentRPM = 0;
    
    public double manualAdjustmentMultiplier = 1;
    
    private final DoubleUnaryOperator distanceToRpm =
        LinearRegression.fit(new double[][]{
//            {150.82, 3428.5},
//            {102.0225, 2957.28}// .495
            //{92.5, 2657.1},
            {58.85, 2442.85},
            {140.73, 3171.42}
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
        targetRPM = Math.round(rpm / RPM_RESOLUTION) * RPM_RESOLUTION;
        
        if (targetRPM > 0 && state == State.OFF) {
            state = State.SPINNING_UP;
        }
    }
    
    public void stop() {
        targetRPM = 0;
        currentTargetRPM = 0;
        totalError = 0;
        currentRPM = 0;
        currentError = 0;
        state = State.OFF;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    
    public void setRpmFromDistance(double dist) {
        setRPM(distanceToRpm.applyAsDouble(dist) * manualAdjustmentMultiplier);
    }
    
    public void readSensors() {
        currentRPM = ticksPerSecondToRpm(rightMotor.getVelocity());
    }
    
    public void update(double dt, double voltage) {
        if (state == State.OFF) {
            return;
        }
        
        currentError = currentTargetRPM - currentRPM;
        double absError = Math.abs(currentError);
        
        state = (absError <= RPM_TOLERANCE)
            ? State.AT_SPEED
            : State.SPINNING_UP;
        
        double targetDiff = targetRPM - currentTargetRPM;
        double maxStep = MAX_ACCEL_RPM_PER_SEC * dt;
        
        if (Math.abs(targetDiff) > maxStep)
            currentTargetRPM += Math.copySign(maxStep, targetDiff);
        else
            currentTargetRPM = targetRPM;
        
        double ff = kS * Math.signum(currentTargetRPM)
            + kV * currentTargetRPM;
        
        double p = kP * currentError;
        
        if (absError < I_ENABLE_ERROR) {
            totalError += currentError * dt;
        }
     
        double i = kI * totalError;
        double power = (ff / voltage + p + i);
        
        power = Range.clip(power, 0, 1);
        
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    
    public double getRPM() {
        return currentRPM;
    }
    
    public double getTargetRPM() {
        return targetRPM;
    }
    
    public double getCurrentError() {
        return currentError;
    }
    
    private double ticksPerSecondToRpm(double tps) {
        return tps / TICKS_PER_REV * 60.0;
    }
    
    public State getState() {
        return state;
    }
    
    public boolean isAtSpeed() {
        return state == State.AT_SPEED;
    }
}
