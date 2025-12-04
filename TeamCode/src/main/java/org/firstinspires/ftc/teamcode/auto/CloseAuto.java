package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;

import java.util.Arrays;

@Autonomous
public class CloseAuto extends Auto2 {
//    public Pose startingPose = new Pose(18.25, 121, -36);
//public Pose startingPose = new Pose(18.25, 121, -36);
    public Pose startingPose = new Pose(20.25, 120.25, -44.5);
    
    // 56.65 87.5, -45.8
    
    // 38.14, 84.98, 180
    public Pose motifPose = new Pose(56.65, 87.5, -95);
    public Pose firePose = new Pose(56.65, 87.5, -45.8);
    
    public Pose pose = new Pose(39, 81, 180);
    public Pose pose2 = new Pose(38, 81-24, 180);
             
             //public Pose motifPose = new Pose(54, 84, -90);
   // public Pose firePose = new Pose(54, 84, -51);

    public Pose endPose = new Pose(30, 50, 180);
    
    MotifDetector motifDetector;
    
    @Override
    public void init() {
        super.init();
        robot.preload(new Artifact[]
                          {Artifact.GREEN,
                              Artifact.PURPLE,
                              Artifact.PURPLE});
        
        motifDetector = new MotifDetector(hardwareMap);
        motifDetector.start();
    }
    
    Timeout timeout = new Timeout();
    
    int state = 1;
    
    @Override
    public void start() {
        super.start();
        
        telemetry.addData("startingPose", startingPose);
        telemetry.update();
        
        robot.follower.setCurrentPose(startingPose);
    }
    
    @Override
    public void loop() {
        switch (state) {
            case 0:
                robot.follower.drivetrain.zeroPower();
                break;
            case 1:
                goToPose(motifPose, Robot.State.REVVING);
                if (state == 2) {
                    timeout.resetAndStart();
                }
                break;
            case 2:
                robot.motifPattern = motifDetector.getMotifPattern();
                if (robot.motifPattern == null && timeout.seconds() > 1) {
                    robot.motifPattern = new Artifact[]
                        {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
                }
                
                if (robot.motifPattern != null || timeout.seconds() > 1) {
                    telemetry.addData("pattern", Arrays.deepToString(robot.motifPattern));
                    telemetry.update();
                    state++;
                }
                break;
            case 7:
           // case 11:
            case 3:
                robot.paddles.close();
                goToPose(firePose, Robot.State.REVVING);
                break;
            case 8:
           // case 12:
            case 4:
                if (robot.state == Robot.State.IDLE) {
                    state++;
                    break;
                }
                else {
                    robot.setState(Robot.State.FAST_FIRING);
                }
                robot.follower.holdPose(firePose);
                break;
            case 5:
                goToPoseFast(pose, Robot.State.IDLE);
                break;
            case 6:
                if (pickupArtifactGroup(81)) {
                    robot.paddles.close();
                    state++;
                }
                break;
            case 9:
                goToPoseFast(pose2, Robot.State.IDLE);
                break;
            case 10:
                if (pickupArtifactGroup(81-24)) {
                    robot.paddles.close();
                    state++;
                }
                break;
            case 11:
                robot.follower.holdPose(endPose);
                break;
        }
        
//        if (time > 30 - 3) {
//            state = 9;
//        }
        
        robot.update();
        
//        telemetry.addData("state", state);
//        telemetry.update();
    }
    
    public void goToPose(Pose pose, Robot.State robotState) {
        robot.follower.holdPose(pose);
        robot.setState(robotState);
        
        if (robot.follower.isStoppedAt(pose)) {
            state++;
        }
    }
    
    public void goToPoseFast(Pose pose, Robot.State robotState) {
        robot.follower.holdPose(pose);
        robot.setState(robotState);
        
        if (robot.follower.isWithinBraking(pose)) {
            state++;
        }
    }
    
    @Override
    public void stop() {
        super.stop();
        motifDetector.stop();
    }
}
