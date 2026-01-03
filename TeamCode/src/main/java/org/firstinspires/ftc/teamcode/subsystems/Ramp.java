package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Ramp {
    ServoImplEx servo;
    
    public Ramp(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(ServoImplEx.class, "intakeRamp");
    }
    
    public void intakeThrough() {
        servo.setPosition(.9);
    } // .935
    
    public void uptake() {
        servo.setPosition(.361);
    }
    
    public void outtake() {
        servo.setPosition(0.725);
    }
    
    public void disable() {
        servo.setPwmDisable();
    }
    
    public void enable() {
        servo.setPwmEnable();
    }
}
