package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.steps.Step;
import org.firstinspires.ftc.teamcode.auto.steps.StepRunner;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;

import java.util.Arrays;

@Autonomous
public class CloseAuto extends Auto {
    public Pose startingPose = new Pose(21.31, 120.26, -35.23);

    public Pose motifPose = new Pose(55.37, 81.82, -95);
    public Pose firePose = new Pose(55.37, 81.82, -44.83);
    
    public Pose pickupPose1 = new Pose(18.25, 79.25, 180);
    public Pose prePickupPose2 = new Pose(42, 79.25-24, 180);
    public Pose pickupPose2 = new Pose(18.25, 79.25-24, 180);
    public Pose prePickupPose3 = new Pose(42, 79.25-48, 180);
    public Pose pickupPose3 = new Pose(18.25, 79.25-48, 180);

    public Pose endPose = new Pose(60, 90, -44.83);
    
    StepRunner auto = new StepRunner();
    
    @Override
    public void init() {
        super.init();
        robot.preload(new Artifact[]
                          {Artifact.GREEN,
                              Artifact.PURPLE,
                              Artifact.PURPLE});
        
        Step fire = new Step(
            () -> robot.follower.holdPose(firePose),
            () -> robot.state == Robot.State.IDLE
        );
        
        auto.add(goToPose(motifPose, Robot.State.REVVING));
        auto.add(detectMotif());
        
        auto.add(goToPose(firePose, Robot.State.REVVING));
        auto.add(fire);
        
        auto.add(goToPose(pickupPose1, Robot.State.CONTINUOUS_INTAKE, .7));
        auto.add(fire);
        auto.add(goToPoseFast(prePickupPose2, Robot.State.IDLE));
        auto.add(goToPose(pickupPose2, Robot.State.CONTINUOUS_INTAKE, .7));
        auto.add(fire);
        auto.add(goToPoseFast(prePickupPose3, Robot.State.IDLE));
        auto.add(goToPose(pickupPose3, Robot.State.CONTINUOUS_INTAKE, .7));
        auto.add(fire);
        
        auto.add(goToPose(endPose, Robot.State.IDLE));
    }
    
    @Override
    public void start() {
        super.start();
        
        telemetry.addData("startingPose", startingPose);
        telemetry.update();
        
        robot.follower.setCurrentPose(startingPose);
    }
    
    
    @Override
    public void loop() {
        robot.update();
        auto.run();
    }
//        switch (state) {
//            case 0:
//                robot.follower.drivetrain.zeroPower();
//                break;
//            case 1:
//                goToPose(motifPose, Robot.State.REVVING);
//                if (state == 2) {
//                    timeout.resetAndStart();
//                }
//                break;
//            case 2:
//                Robot.motifPattern = motifDetector.getMotifPattern();
//                if (Robot.motifPattern == null && timeout.seconds() > 1) {
//                    Robot.motifPattern = new Artifact[]
//                        {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
//                }
//
//                if (Robot.motifPattern != null || timeout.seconds() > 1) {
//                    telemetry.addData("pattern", Arrays.deepToString(Robot.motifPattern));
//                    telemetry.update();
//                    state++;
//                }
//                break;
//            case 6:
//            case 9:
//            case 12:
//            case 3:
//                goToPose(firePose, Robot.State.REVVING);
//                break;
//            case 7:
//            case 10:
//            case 13:
//            case 4:
//                if (robot.state == Robot.State.IDLE) {
//                    state++;
//                    break;
//                }
//                else {
//                    robot.setState(Robot.State.FIRING);
//                }
//                robot.follower.holdPose(firePose);
//                break;
//            case 5:
//                if (timeout.seconds() > 5 || robot.spindexer.getNumberOfArtifacts() == 3) {
//                    state++;
//                    break;
//                }
//                goToPose(pickupPose1, Robot.State.CONTINUOUS_INTAKE);
//                break;
//            case 8:
//                goToPoseFast(pickupPose2, Robot.State.CONTINUOUS_INTAKE);
//                break;
//            case 11:
//                goToPoseFast(pickupPose3, Robot.State.CONTINUOUS_INTAKE);
//                break;
//            case 14:
//                goToPose(endPose, Robot.State.IDLE);
//                break;
//        }

//
//        robot.update();
        
//        telemetry.addData("state", state);
//        telemetry.update();
    //}
    
//    public void goToPose(Pose pose, Robot.State robotState) {
//        robot.follower.holdPose(pose);
//        robot.setState(robotState);
//
//        if (robot.follower.isStoppedAt(pose)) {
//            state++;
//        }
//    }
//
//    public void goToPoseFast(Pose pose, Robot.State robotState) {
//        robot.follower.holdPose(pose);
//        robot.setState(robotState);
//
//        if (robot.follower.isWithinBraking(pose)) {
//            state++;
//        }
//    }
    
    @Override
    public void stop() {
        super.stop();
        motifDetector.stop();
    }
}
