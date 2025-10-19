package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx motor;
    
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void intake() {
        motor.setPower(1);
    }
    
    public void outtake() {
        motor.setPower(-0.5);
    }
    
    public void spinIntoShooter() {
        motor.setPower(-0.5);
    }
    
    public void stop() {
        motor.setPower(0);
    }
    
    public void maxPowerIntake() {
        motor.setPower(1);
    }
}
