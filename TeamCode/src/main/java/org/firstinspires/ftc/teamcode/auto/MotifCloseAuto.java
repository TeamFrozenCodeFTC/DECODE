package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.steps.Step;
import org.firstinspires.ftc.teamcode.auto.steps.StepRunner;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class MotifCloseAuto extends Auto {
    // 16.5, 17 + 3/4
    public Pose startingPose = new Pose(19, 119.61, -36.42);
    
    public Pose motifPose = new Pose(55.37, 81.82, -95);
    public Pose firePose = new Pose(55.37, 81.82, -44.83);
    
    public Pose prePickupPose1 = new Pose(43, 81, 180);
    public Pose pickupPose1 = new Pose(19, 81, 180);
    public Pose prePickupPose2 = new Pose(43, 58.5, 180);
    public Pose pickupPose2 = new Pose(18, 79.25-24, 180);
    public Pose prePickupPose3 = new Pose(43, 79.25-48, 180);
    public Pose pickupPose3 = new Pose(18, 33.5, 180);
    
    public Pose endPose = new Pose(60, 90, -44.83);
    
    StepRunner auto = new StepRunner();
    
    public Step getFireStep() {
        return new Step(
            () -> robot.setState(Robot.State.MOTIF_FIRING),
            () -> robot.follower.holdPose(firePose),
            () -> robot.state == Robot.State.IDLE
        );
    }
    
    @Override
    public void init() {
        super.init();
        robot.preload(new Artifact[]
                          {Artifact.GREEN,
                              Artifact.PURPLE,
                              Artifact.PURPLE});
        
        auto.add(goToPose(motifPose, Robot.State.REVVING));
        auto.add(detectMotif());
        auto.add(goToPose(firePose, Robot.State.REVVING));
        auto.add(getFireStep());
        auto.add(goToPose(prePickupPose1, Robot.State.IDLE));
        auto.add(goToPose(pickupPose1, Robot.State.CONTINUOUS_INTAKE, .5));
        auto.add(Step.timeout(1));
        auto.add(goToPose(firePose, Robot.State.REVVING));
        auto.add(getFireStep());
        auto.add(goToPoseFast(prePickupPose2, Robot.State.IDLE));
        auto.add(goToPose(pickupPose2, Robot.State.CONTINUOUS_INTAKE, .5));
        auto.add(Step.timeout(1));
        auto.add(goToPose(firePose, Robot.State.REVVING));
        auto.add(getFireStep());
        auto.add(goToPoseFast(prePickupPose3, Robot.State.IDLE));
        auto.add(goToPose(pickupPose3, Robot.State.CONTINUOUS_INTAKE, .5));
        auto.add(Step.timeout(1));
        auto.add(goToPose(firePose, Robot.State.REVVING));
        auto.add(getFireStep());
        
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
        
        telemetry.addData("pose", robot.follower.localizer.getPose());
        telemetry.addData("state", robot.state);
        telemetry.addData("numOfArtifacts", robot.spindexer.getNumberOfArtifacts());
        telemetry.update();
    }
    @Override
    public void stop() {
        super.stop();
        motifDetector.stop();
    }
}
