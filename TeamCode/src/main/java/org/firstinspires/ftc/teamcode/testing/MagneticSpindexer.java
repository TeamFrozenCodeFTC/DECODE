package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.blackice.util.Timeout;

@Autonomous
public class MagneticSpindexer extends OpMode {
    CRServo servo;
    TouchSensor magnetSensor;
    
    boolean wasPressed = false;
    
    boolean otherDirection = false;
    
    @Override
    public void init() {
        servo = hardwareMap.get(CRServoImplEx.class, "spindexer");
        magnetSensor = hardwareMap.get(TouchSensor.class, "magnetSensor");
    }
    
    double times = 1;
    Timeout timeout = new Timeout();
    
    @Override
    public void loop() {
         if (magnetSensor.isPressed() && !wasPressed) {
             wasPressed = true;
             servo.setPower(-1);
             timeout.resetAndStart();
         }
         
         if (wasPressed) {
             if (timeout.seconds() > 0.1) {
                 servo.setPower(0);
             }
             else {
                 servo.setPower(-1);
             }
         }
         else {
             servo.setPower(1);
         }
    }
}
