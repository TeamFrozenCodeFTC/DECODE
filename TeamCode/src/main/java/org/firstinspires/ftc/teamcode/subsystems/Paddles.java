package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Paddles {
    ServoImplEx leftPaddle;
    ServoImplEx rightPaddle;
    
    public Paddles(HardwareMap hardwareMap) {
        this.leftPaddle = hardwareMap.get(ServoImplEx.class, "leftPaddle");
        this.rightPaddle = hardwareMap.get(ServoImplEx.class, "rightPaddle");
    }
    
    public void open() {
        leftPaddle.setPosition(0.823);
        rightPaddle.setPosition(0.175);
    }
    
    public void close() {
        leftPaddle.setPosition(0.48);
        rightPaddle.setPosition(0.5);
    }
}
