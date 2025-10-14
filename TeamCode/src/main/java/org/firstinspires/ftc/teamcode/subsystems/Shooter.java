package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {
    private final DcMotorEx rightMotor;
    private final DcMotorEx leftMotor;
    public static double RADIUS = 2;
    public static double REV_DELTA = 0.01;
    double currentPower = 0;
    
    private boolean isRevingUp = false;
    private double targetPower = 0;
    
    public Shooter(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    
    public void setPower(double power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
        currentPower = power;
    }
    
    public double getCurrentPower() {
        return currentPower;
    }
    
    // try pid to maintain velocity
    // graph velocity first to see how it behaves with artifacts
    // test pushing artifacts in too
    
    public void update(double voltageCompensation) {
        if (isRevingUp) {
            currentPower = Math.min(targetPower, currentPower + REV_DELTA);
            isRevingUp = false;
        }
        setPower(currentPower * voltageCompensation);
    }
    
    /**
     * Gradually revs up the flywheel to the specified power.
     */
    public void revUpShooterTo(double power) {
        isRevingUp = true;
        targetPower = power;
    }

    public double getAngularVelocity() {
        return (leftMotor.getVelocity(AngleUnit.RADIANS) + rightMotor.getVelocity(AngleUnit.RADIANS)) / 2;
    }
    
    public double getRPM() {
        return getAngularVelocity() / (2 * Math.PI);
    }
    
    public double getVelocity() {
        return getAngularVelocity() * RADIUS;
    }
    
    public boolean isUpToSpeed() {
        return getRPM() > 5000;
    }
    
    public void uptake() {
        setPower(-0.2);
    }
    
    public void stop() {
        setPower(0);
    }
}
