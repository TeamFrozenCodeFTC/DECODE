package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Shooter {
    public final DcMotorEx rightMotor;
    public final DcMotorEx leftMotor;
    
    private double targetRPM = 0;
    private double currentTargetRPM = 0;
    
    public final int TICKS_PER_REV = 28;
    public final double MAX_DISTANCE_TO_GOAL = 136.82;
    public final double MAX_RPM_REQUIRED = 5000;
    public final double ACCELERATION = 2000; // rpm per second
    
    PIDFCoefficients coefficients = new PIDFCoefficients(10, 3, 0, 0);

    public void updateFeedforwardByVoltage(double voltageCompensation) {
        PIDFCoefficients pidf = new PIDFCoefficients(coefficients.p, coefficients.i,
                                                     coefficients.d, coefficients.f * voltageCompensation);
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public void updateCoefficients() {
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }
    
    public Shooter(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        updateCoefficients();
    }
    
    public double getTargetRPM() {
        return targetRPM;
    }
    
    public double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV / 60;
    }
    
    public double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond / TICKS_PER_REV * 60;
    }
    
    public void setRPM(double rpm) {
        targetRPM = rpm;
    }
    
    public void setRpmFromDistance(double distanceToGoal) {
        double rpm = (distanceToGoal / MAX_DISTANCE_TO_GOAL) * MAX_RPM_REQUIRED;
        setRPM(rpm);
    }
    
    public void update(double deltaTime) {
        double rpmDifference = targetRPM - currentTargetRPM;
        double maxStep = ACCELERATION * deltaTime;
        
        if (Math.abs(rpmDifference) > maxStep) {
            currentTargetRPM += Math.copySign(maxStep, rpmDifference);
        } else {
            currentTargetRPM = targetRPM;
        }
        
        double ticksPerSecond = rpmToTicksPerSecond(currentTargetRPM);
        rightMotor.setVelocity(ticksPerSecond);
        leftMotor.setVelocity(ticksPerSecond);
    }

    public double getTicksPerSecond() {
        return (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2;
    }
    
    public double getRpm() {
        return ticksPerSecondToRpm(getTicksPerSecond());
    }

    public boolean isUpToSpeed() {
        return Math.abs(getRpm() - targetRPM) < 50; // within 50 RPM tolerance
    }
    
    public void uptake() {
        setRPM(-435);
    }
    
    public void stop() {
        setRPM(0);
    }
}
