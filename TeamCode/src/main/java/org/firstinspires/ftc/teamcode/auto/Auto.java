package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.steps.Step;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;

import java.lang.reflect.Field;

public abstract class Auto extends OpMode {
    public Robot robot;
    
    MotifDetector motifDetector;
    
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.spindexer.rotateToSlot(0.5);
        robot.isAuto = true;
        motifDetector = new MotifDetector(hardwareMap);
        motifDetector.start();
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
    
    public Step goToPose(Pose pose, Robot.State state) {
        return new Step(
            () -> robot.setState(state),
            () -> robot.follower.holdPose(pose),
            () -> robot.follower.isStoppedAt(pose)
        );
    }
    
    public Step goToPoseFast(Pose pose, Robot.State state) {
        return new Step(
            () -> robot.setState(state),
            () -> robot.follower.holdPose(pose),
            () -> robot.follower.isWithinBraking(pose)
        );
    }
    
    public Step goToPose(Pose pose, Robot.State state, double power) {
        return new Step(
            () -> robot.setState(state),
            () -> robot.follower.holdPose(pose, power),
            () -> robot.follower.isStoppedAt(pose)
        );
    }
    
    public Step detectMotif() {
        ElapsedTime timeout = new ElapsedTime();
        
        return new Step(
            timeout::reset,
            () -> {
                Robot.motifPattern = motifDetector.getMotifPattern();
                if (Robot.motifPattern == null && timeout.seconds() > 1) {
                    Robot.motifPattern = new Artifact[]{
                        Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE
                    };
                }
            },
            () -> Robot.motifPattern != null || timeout.seconds() > 1
        );
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
    
    int artifactToPickUp = 1; // 1,2,3
    double pickupStartTime = -1;
    boolean pickingUpArtifact = false;
    ElapsedTime timer;
    
//    public boolean pickupArtifactGroup(double y) {
//        robot.setState(Robot.State.CONTINUOUS_INTAKE);
//
//        Pose pickup = getPose(y);
//
//        if (pickupStartTime < 0) {
//            pickupStartTime = time;
//        }
//
//        double elapsed = time - pickupStartTime;
//        if (robot.intakedArtifact.isArtifact()) { // Stop while intaking artifact
//            if (!pickingUpArtifact) {
//                robot.follower.drivetrain.zeroPower();
//                artifactToPickUp++;
//                pickupStartTime = time;
//                pickingUpArtifact = true;
//            }
//        }
//        else if (artifactToPickUp > 3) {
//            pickingUpArtifact = false;
//            artifactToPickUp = 1;
//            pickupStartTime = -1;
//            return true;
//        }
//        else if (elapsed > 5) { // Skip to next artifact
//            artifactToPickUp++;
//            pickupStartTime = time;
//            pickingUpArtifact = false;
//        }
//        else if (elapsed > 1.2) { // Move forward to try to get artifact
//            pickingUpArtifact = false;
//            robot.follower.drivetrain.followVector(new Vector(0.25, 0), 0);
//        }
//        else { // Move to next artifact
//            pickingUpArtifact = false;
//            robot.follower.holdPose(pickup, 0.3);
//        }
//
//        return false;
//    }
public boolean pickupArtifactGroup(double y) {
    robot.setState(Robot.State.CONTINUOUS_INTAKE);
    
    Pose pickup = getPose(y);
    
    if (pickupStartTime < 0) {
        pickupStartTime = time;
    }
    
    robot.follower.holdPose(pickup);
    
    return robot.follower.isStoppedAt(pickup);

//    double elapsed = time - pickupStartTime;
//    if (robot.intakedArtifact.isArtifact()) { // Stop while intaking artifact
//        if (!pickingUpArtifact) {
//            robot.follower.drivetrain.zeroPower();
//            artifactToPickUp++;
//            pickupStartTime = time;
//            pickingUpArtifact = true;
//        }
//    }
//    else if (artifactToPickUp > 3) {
//        pickingUpArtifact = false;
//        artifactToPickUp = 1;
//        pickupStartTime = -1;
//        return true;
//    }
//    else if (elapsed > 5) { // Skip to next artifact
//        artifactToPickUp++;
//        pickupStartTime = time;
//        pickingUpArtifact = false;
//    }
//    else if (elapsed > 1.2) { // Move forward to try to get artifact
//        pickingUpArtifact = false;
//        robot.follower.drivetrain.followVector(new Vector(0.25, 0), 0);
//    }
//    else { // Move to next artifact
//        pickingUpArtifact = false;
//        robot.follower.holdPose(pickup, 0.3);
//    }
//
//    return false;
}
    
    private Pose getPose(double y) {
        double x;
        switch (artifactToPickUp) {
            
            // 38.14, 84.98, 180
//            case 1: x = 37.25; break;
//            case 2: x = 32; break;
//            case 3: x = 24; break;
            case 1: x = 38.14; break;
            case 2: x = 38.14 - 5; break;
            //case 3: x = 38.14 - 10; break;
            default: x = 38.14 - 10; break;
//            default:
//                pickingUpArtifact = false;
//                artifactToPickUp = 0;
//                return true;
        }
        
        Pose pickup = new Pose(x, y, 180);
        if (Robot.allianceColor == AllianceColor.RED) {
            pickup = pickup.mirroredAcrossYAxis();
        }
        return pickup;
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
