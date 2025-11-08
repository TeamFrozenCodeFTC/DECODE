package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class Auto2 extends OpMode {
    Robot robot;

    public AllianceColor allianceColor = AllianceColor.BLUE;
    public static Pose startingPose = new Pose(24*3+12,
                                               144-((double) 17/2), 180);
    
//    Menu startingStateMenu;
//
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        
//        startingStateMenu = new Menu(gamepad1);
//        for (Map<String, PathRoutine> map : getAllAccessiblePathRoutines()) {
//            for (String name : map.keySet()) {
//                PathRoutine routine = map.get(name);
//                startingStateMenu.addOption(name, () -> {}, () -> currentRoutine = routine);
//            }
//        }
//        startingStateMenu.confirmOption(0);
        
        robot.spindexer.slots = new Spindexer.Artifact[]{
            Spindexer.Artifact.PURPLE,
            Spindexer.Artifact.PURPLE,
            Spindexer.Artifact.PURPLE
        };
    }
    
    @Override
    public void init_loop() {
//        startingStateMenu.update();
//
        if (gamepad1.triangleWasPressed()) {
            allianceColor = AllianceColor.BLUE == allianceColor ? AllianceColor.RED : AllianceColor.BLUE;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        telemetry.addData("Alliance Color (Press △)", allianceColor);
        telemetry.update();
    }
    
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(startingPose);
    }
    
    @Override
    public void stop() {
        blackboard.put("currentPose", robot.follower.getCurrentPose());
        blackboard.put("allianceColor", allianceColor);
    }
}
