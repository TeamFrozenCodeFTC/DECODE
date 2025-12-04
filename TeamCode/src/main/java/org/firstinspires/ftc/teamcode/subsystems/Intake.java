package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public final DcMotorEx motor;
    
    private double targetPower = 0;
    private double currentPower = 0;
    private final double rampRate = 0.1;
    
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void update() {
        if (Math.abs(targetPower - currentPower) < rampRate) {
            currentPower = targetPower;
        } else if (currentPower < targetPower) {
            currentPower += rampRate;
        } else {
            currentPower -= rampRate;
        }
        
        motor.setPower(currentPower);
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
