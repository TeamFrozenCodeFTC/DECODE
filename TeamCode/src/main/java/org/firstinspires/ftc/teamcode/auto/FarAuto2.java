package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.steps.Step;
import org.firstinspires.ftc.teamcode.auto.steps.StepRunner;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

import java.util.Arrays;

@Autonomous
public class FarAuto2 extends Auto {
    // robot size = (17.75, 16)

    public Pose startingPose = new Pose(56, 17.75/2, -90);
    public Pose firePose = new Pose(57, 17, -65);
    
    public Pose pickupPose = new Pose(17.75/2, 16/2, -180);
    
    public Pose endPose = new Pose(51, 21, -46);
    
    StepRunner auto = new StepRunner();
    
    @Override
    public void init() {
        super.init();
        robot.preload(new Artifact[]
                          {Artifact.GREEN,
                              Artifact.PURPLE,
                              Artifact.PURPLE});
        
        auto.add(goToPose(firePose, Robot.State.REVVING));
        auto.add(getFireStep(firePose));
        
        auto.add(goToPose(endPose, Robot.State.IDLE));
    }
    
    @Override
    public void start() {
        super.start();
        
//        Robot.motifPattern = motifDetector.getMotifPattern();
//        if (Robot.motifPattern == null) {
//            Robot.motifPattern = new Artifact[]
//                {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
//        }
//        telemetry.addData("pattern", Arrays.deepToString(Robot.motifPattern));
        telemetry.addData("startingPose", startingPose);
        telemetry.update();
        
        robot.follower.setCurrentPose(startingPose);
    }
    
    @Override
    public void loop() {
        robot.update();
        auto.run();
        
//        telemetry.addData("pose", robot.follower.localizer.getPose());
//        telemetry.addData("state", robot.state);
//        telemetry.addData("numOfArtifacts", robot.spindexer.getNumberOfArtifacts());
//        telemetry.update();
    }
    
    @Override
    public void stop() {
        super.stop();
        motifDetector.stop();
    }
}
