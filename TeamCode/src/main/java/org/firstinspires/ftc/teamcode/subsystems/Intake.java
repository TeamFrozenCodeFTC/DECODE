package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public final DcMotorEx motor;
    
    private double targetPower = 0;
    private double currentPower = 0;
    private final double rampRate = 3; // power per second
    
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void update(double deltaTime) {
        if (targetPower > currentPower) {
            currentPower = Math.min(currentPower + rampRate * deltaTime, targetPower);
        } else {
            currentPower = targetPower;
        }
        
        motor.setPower(currentPower);
    }
    
    public void setTargetPower(double power) {
        targetPower = power;
    }
    
    public double getTargetPower() {
        return targetPower;
    }
    
    public void intake() {
        targetPower = 1;
    }
    
    public void outtake() {
        targetPower = -1;
    }
    
    public void stop() {
        targetPower = 0;
    }
}
