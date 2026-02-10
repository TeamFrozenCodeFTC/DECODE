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

public abstract class Auto extends OpMode {
    public Robot robot;
    
    //MotifDetector motifDetector;
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.spindexer.rotateToSlot(0.5);
        robot.isAuto = true;
        
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
    
    public Step getFireStep(Pose firePose) {
        return new Step(
            () -> robot.setState(Robot.State.LAUNCHING),
            () -> robot.follower.holdPose(firePose),
            () -> robot.state == Robot.State.IDLE,
            () -> robot.spindexer.artifacts = Artifact.getEmptyPattern()
        ).withTimeout(4); // was 3
    }
    
    public Step getFireStep2(Pose firePose) {
        return new Step(
            () -> robot.setState(Robot.State.LAUNCHING),
            () -> robot.follower.holdPose(firePose),
            () -> robot.state == Robot.State.IDLE,
            () -> robot.spindexer.artifacts = Artifact.getEmptyPattern()
        ).withTimeout(4); // was 3
    }
    
    public Step driveIntoArtifacts(Pose pose) {
        return new Step(
            () -> robot.setState(Robot.State.INTAKING),
            () -> robot.follower.holdPose(pose),
            () -> robot.spindexer.getNumberOfArtifacts() == 3
        ).withTimeout(4);
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
    
    
    public Step goToPoseFast(Pose pose, Robot.State state, double power) {
        return new Step(
            () -> robot.setState(state),
            () -> robot.follower.holdPose(pose, power),
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
//
//    public Step detectMotif() {
//        ElapsedTime timeout = new ElapsedTime();
//
//        return new Step(
//            timeout::reset,
//            () -> {
//                Robot.motifPattern = motifDetector.getMotifPattern();
//                if (Robot.motifPattern == null && timeout.seconds() > 1) {
//                    Robot.motifPattern = new Artifact[]{
//                        Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE
//                    };
//                }
//            },
//            () -> Robot.motifPattern != null || timeout.seconds() > 1
//        );
//    }
    
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
    
//    @Override
//    public void stop() {
//        motifDetector.stop();
//        robot.spindexer.distanceSensors.close();
//    }
}
