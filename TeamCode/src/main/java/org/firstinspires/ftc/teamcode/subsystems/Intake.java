package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Intake {
    private static final double rampRate = 5; // power units per second
    
    public final DcMotorEx motor;
    
    public enum State {
        OFF,
        INTAKING,
        REVERSING
    }
    
    private State currentState = State.OFF;
    
    private double targetPower = 0;
    private double currentPower = 0;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void update(double deltaTime) {
        if (currentPower < targetPower) {
            currentPower = Math.min(currentPower + rampRate * deltaTime, targetPower);
        } else {
            currentPower = targetPower;
        }
        
        motor.setPower(currentPower);
    }
    
    public void setTargetPower(double power) {
        targetPower = Range.clip(power, -1, 1);
    }
    
    public double getTargetPower() {
        return targetPower;
    }
    
    public void intake() {
        targetPower = 1;
        currentState = State.INTAKING;
    }
    
    public void outtake() {
        targetPower = -1;
        currentState = State.REVERSING;
    }
    
    public void stop() {
        targetPower = 0;
        currentState = State.OFF;
    }
    
    public State getState() {
        return currentState;
    }
}
