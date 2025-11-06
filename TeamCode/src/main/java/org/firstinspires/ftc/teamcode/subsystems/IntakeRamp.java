package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class IntakeRamp {
    ServoImplEx servo;
    
    public IntakeRamp(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(ServoImplEx.class, "intakeRamp");
    }
    
    public void intakeThrough() {
        servo.setPosition(.933);
    }
    
    public void uptake() {
        servo.setPosition(0.423);
    }
    
    public void outtake() {
        servo.setPosition(0.654);
    }
    
}
