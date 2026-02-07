package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.steps.Step;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;

import java.lang.reflect.Field;

public abstract class Auto2 extends OpMode {
    public Robot robot;
    
   // MotifDetector motifDetector;
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.spindexer.rotateToSlot(0.5);
        robot.isAuto = true;
//
//        motifDetector = new MotifDetector(hardwareMap);
//        motifDetector.start();
    }
    
    @Override
    public void init_loop() {
        if (gamepad1.triangleWasPressed()) {
            Robot.allianceColor = (AllianceColor.BLUE == Robot.allianceColor) ?
                AllianceColor.RED :
                AllianceColor.BLUE;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        telemetry.addData("Alliance Color (Press △)", Robot.allianceColor);
        telemetry.update();
    }
    
    
    @Override
    public void start() {
        if (Robot.allianceColor == AllianceColor.RED) {
            mirrorPosesForAllianceColor();
        }
    }
    
    @Override
    public void loop() {
        robot.update();
    }
    
    public void mirrorPosesForAllianceColor() {
        for (Field field : getClass().getDeclaredFields()) {
            if (!Pose.class.isAssignableFrom(field.getType())) continue;
            
            field.setAccessible(true);
            try {
                Pose pose = (Pose) field.get(this);
                if (pose != null) {
                    field.set(this, pose.mirroredAcrossYAxis());
                }
            } catch (IllegalAccessException e) {
                telemetry.addLine("Cannot access " + field.getName());
            }
        }
    }
}
