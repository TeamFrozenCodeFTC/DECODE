package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.spindexer.MotifPattern;
import org.firstinspires.ftc.teamcode.utils.DelayWrapper;

@Autonomous
public class CloseAuto2 extends Auto2 {
    Pose startingPose = new Pose(33, 135.125, -90);
    //Pose launchingPose = new Pose(50, 87, -48);
    Pose launchingPose = new Pose(55, 84, -48);
    
    Pose pickUpPose1ControlPoint = new Pose(45, 84+3, 180);
    Pose pickUpPose1 = new Pose(16, 84+3, 180);
    
    Pose pickUpPose2ControlPoint = new Pose(50, 54+1, 180);
    Pose pickUpPose2 = new Pose(9, 60+1, 180);
    
    Pose prePickup3 = new Pose(42, 36+3, 180);
    Pose pickUpPose3 = new Pose(9, 36+3, 180);
    
//    Pose openGatePosition = new Pose(22, 67, 240);
    Pose openGatePosition = new Pose(22, 69, 240);
    Pose endPose = new Pose(50, 76, -44.83);

    AutoRoutine autoRoutine;
    
    @Override
    public void init() {
        super.init();
        
        robot.preload(
            new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE});
        
        robot.follower.setTelemetry(telemetry);
        
        robot.transfer.feedFromSpindexer();
        
    }
    
    public AutoRoutine fireArtifacts(Pose startingPose) {
        return robot.follower.autoBuilder(startingPose)
            .addAction(() -> robot.setState(Robot.State.REVVING))
            .lineTo(launchingPose).stop()
            .addAction(() -> robot.setState(Robot.State.LAUNCHING))
            .holdLastPath()
                .until(new DelayWrapper(350, () -> robot.state == Robot.State.IDLE))
            .build();
    }
    
    @Override
    public void start() {
        super.start();
        
        robot.follower.setCurrentPose(startingPose);

        autoRoutine = robot.follower.autoBuilder(startingPose)
            .addRoutine(fireArtifacts(startingPose))
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .curveTo(pickUpPose1ControlPoint, pickUpPose1)
            .until(() -> {
                MotifPattern pattern = motifDetector.getMotifPattern();
                if (pattern != null) {
                    telemetry.addData("Motif", pattern);
                    Robot.motifPattern = pattern;
                }

                return robot.spindexer.getNumberOfArtifacts() == 3;
            })
                .withTimeout(5.5)
            //.withTimeout(4)
            
            .addRoutine(fireArtifacts(pickUpPose1))
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .curveTo(pickUpPose2ControlPoint, pickUpPose2)
            .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            .withTimeout(6)
            
            .lineTo(openGatePosition)
            
            .addRoutine(fireArtifacts(openGatePosition))
            
            .lineTo(prePickup3)
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(pickUpPose3, 5)
            .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            
            .addRoutine(fireArtifacts(pickUpPose3))
                .lineTo(endPose)
                .stop()
            .build();
        
        autoRoutine.start();
    }
    
    boolean foundPattern = false;
    
    @Override
    public void stop() {
        motifDetector.stop();
        robot.spindexer.distanceSensors.close();
    }
    
    @Override
    public void loop() {
        super.loop();
        
        autoRoutine.run();
//
//        if (autoRoutine.getIndex() == 5 && !foundPattern) {
//            MotifPattern pattern = motifDetector.getMotifPattern();
//            if (pattern != null) {
//                Robot.motifPattern = pattern;
//                foundPattern = true;
//            }
//        }
        
//        telemetry.addData("index", autoRoutine.getIndex());
//        telemetry.addData("state", robot.state);
//        telemetry.addData("pose", robot.follower.getCurrentPose());
//        telemetry.update();
    }
}
