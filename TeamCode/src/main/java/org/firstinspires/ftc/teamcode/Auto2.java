package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;

import java.lang.reflect.Field;

public abstract class Auto2 extends OpMode {
    Robot robot;

    public AllianceColor allianceColor = AllianceColor.BLUE;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());

    }
    
    @Override
    public void init_loop() {
        if (gamepad1.triangleWasPressed()) {
            allianceColor = (AllianceColor.BLUE == allianceColor) ? AllianceColor.RED :
                AllianceColor.BLUE;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        telemetry.addData("Alliance Color (Press △)", allianceColor);
        telemetry.update();
    }
    
    @Override
    public void start() {
        if (allianceColor == AllianceColor.RED) {
            mirrorPosesForAllianceColor();
        }
    }
    
    @Override
    public void stop() {
        blackboard.put("currentPose", robot.follower.getCurrentPose());
        blackboard.put("allianceColor", allianceColor);
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



//        startingStateMenu = new Menu(gamepad1);
//        for (Map<String, PathRoutine> map : getAllAccessiblePathRoutines()) {
//            for (String name : map.keySet()) {
//                PathRoutine routine = map.get(name);
//                startingStateMenu.addOption(name, () -> {}, () -> currentRoutine = routine);
//            }
//        }
//        startingStateMenu.confirmOption(0);
//        startingStateMenu.update();
