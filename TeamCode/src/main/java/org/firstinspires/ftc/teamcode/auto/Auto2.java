package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;

import java.lang.reflect.Field;

public abstract class Auto2 extends OpMode {
    public Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.spindexer.rotateToSlot(0.5);
    }
    
    @Override
    public void init_loop() {
        if (gamepad1.triangleWasPressed()) {
            robot.allianceColor = (AllianceColor.BLUE == robot.allianceColor) ?
                AllianceColor.RED :
                AllianceColor.BLUE;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        telemetry.addData("Alliance Color (Press △)", robot.allianceColor);
        telemetry.update();
    }
    
    @Override
    public void start() {
        if (robot.allianceColor == AllianceColor.RED) {
            mirrorPosesForAllianceColor();
        }
    }
    
    @Override
    public void stop() {
        blackboard.put("currentPose", robot.follower.getCurrentPose());
        blackboard.put("allianceColor", robot.allianceColor);
        blackboard.put("motifPattern", robot.motifPattern);
    }
    
    @Override
    public void loop() {
        robot.update(telemetry);
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



//        startingStateMenu = new Menu(gamepad1);
//        for (Map<String, PathRoutine> map : getAllAccessiblePathRoutines()) {
//            for (String name : map.keySet()) {
//                PathRoutine routine = map.get(name);
//                startingStateMenu.addOption(name, () -> {}, () -> currentRoutine = routine);
//            }
//        }
//        startingStateMenu.confirmOption(0);
//        startingStateMenu.update();
