package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Autonomous
public class Auto extends OpMode {
    Robot robot;
    private PathRoutine currentRoutine;
    
    PathRoutine shootPreload;
    PathRoutine toArtifactGroup1;
    PathRoutine toArtifactGroup2;
    PathRoutine toArtifactGroup3;
    PathRoutine shootArtifactGroup1;
    PathRoutine shootArtifactGroup2;
    PathRoutine shootArtifactGroup3;
    
    public static final Pose startingPose = new Pose(24*3+12,
                                                     144-((double) 17/2), 180);
    public static final Pose artifactGroup1Pose = new Pose(104-2, 3.5*24, 180);
    public static final Pose shootArtifactGroup1Pose = new Pose(108, 94, -118);
    
    public AllianceColor allianceColor = AllianceColor.BLUE;
    
    public enum AllianceColor {
        RED,
        BLUE
    }
    
    public void buildPaths() {
        shootPreload = robot.follower.pathRoutineBuilder()
            .runAction(robot::shootArtifacts)
            .build();
        
        toArtifactGroup1 = robot.follower.pathRoutineBuilder()
            .toPose(artifactGroup1Pose)
            .withHeadingInterpolator(pathPoint -> {
                if (pathPoint.percentAlongPath > 0.20) {
                    return 0;
                }
                else {
                    return 180;
                }
            })
            .runAction(robot::intake)
            .toPose(artifactGroup1Pose.addedX(18))
            .build();
        
        shootArtifactGroup1 = robot.follower.pathRoutineBuilder()
            .toPose(shootArtifactGroup1Pose)
            .runAction(robot::shootArtifacts)
            .build();
    }
    
    Menu startingStateMenu;
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.follower.setCurrentPose(startingPose);  // pretty sure this method was
        // broken

        startingStateMenu = new Menu(gamepad1);
        for (Map<String, PathRoutine> map : getAllAccessiblePathRoutines()) {
            for (String name : map.keySet()) {
                PathRoutine routine = map.get(name);
                startingStateMenu.addOption(name, () -> {}, () -> currentRoutine = routine);
            }
        }
        
        robot.indexer.slots = new Indexer.Slot[]{
            Indexer.Slot.PURPLE,
            Indexer.Slot.PURPLE,
            Indexer.Slot.PURPLE
        };
        
        buildPaths();
    }
    
    @Override
    public void init_loop() {
        if (gamepad1.triangleWasPressed()) {
            allianceColor = AllianceColor.BLUE == allianceColor ? AllianceColor.RED : AllianceColor.BLUE;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        telemetry.addData("Alliance Color (Press △) - ", allianceColor);
        telemetry.addData("Starting Position - ", robot.follower.getCurrentPose());
        telemetry.addData("Starting State - ", startingStateMenu.getSelectedOption());
        telemetry.addLine(startingStateMenu.getDisplay());
        
        startingStateMenu.update();
        
        telemetry.update();
    }
  
    void setRoutine(PathRoutine nextRoutine) {
        currentRoutine = nextRoutine;
        robot.follower.follow(currentRoutine);
    }
    
    @Override
    public void start() {
        setRoutine(shootPreload);
    }
    
    @Override
    public void loop() {
        if (currentRoutine == shootPreload) {
            if (!robot.isShooting) {
                setRoutine(toArtifactGroup1);
            }
        }
        if (currentRoutine == toArtifactGroup1) {
            if (!robot.follower.isInProgress()) {
                setRoutine(shootArtifactGroup1);
            }
        }
        if (currentRoutine == shootArtifactGroup1) {
            if (!robot.isShooting) {
                setRoutine(toArtifactGroup2);
            }
        }
        
        robot.update();
        
        telemetry.addData("routine", currentRoutine.getClass().getSimpleName());
    }
    
    public List<Map<String, PathRoutine>> getAllAccessiblePathRoutines() {
        List<Map<String, PathRoutine>> routineList = new ArrayList<>();
        
        for (Field field : getClass().getDeclaredFields()) {
            if (PathRoutine.class.isAssignableFrom(field.getType())) {
                field.setAccessible(true);
                try {
                    PathRoutine routine = (PathRoutine) field.get(this);
                    if (routine != null) {
                        Map<String, PathRoutine> entry = new HashMap<>();
                        entry.put(field.getName(), routine); // variable name -> object
                        routineList.add(entry);
                    }
                } catch (IllegalAccessException e) {
                    telemetry.addLine("Cannot access " + field.getName());
                }
            }
        }
        
        return routineList;
    }
    
    @Override
    public void stop() {
        blackboard.put("currentPose", robot.follower.getCurrentPose());
        blackboard.put("allianceColor", allianceColor);
    }
}
