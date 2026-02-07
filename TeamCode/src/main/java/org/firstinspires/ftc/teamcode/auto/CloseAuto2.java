package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class CloseAuto2 extends Auto2 {
    Pose startingPose = new Pose(33, 135.125, -90);
    Pose launchingPose = new Pose(50, 87, -48);
    
    Pose pickUpPose1ControlPoint = new Pose(45, 84, 180);
    Pose pickUpPose1 = new Pose(16, 84, 180);
    
    Pose pickUpPose2ControlPoint = new Pose(50, 54, 180);
    Pose pickUpPose2 = new Pose(10, 60, 180);
    
    Pose prePickup3 = new Pose(42, 36, 180);
    Pose pickUpPose3 = new Pose(10, 36, 180);
    
//    Pose openGatePosition = new Pose(22, 67, 240);
    Pose openGatePosition = new Pose(22, 69, 240);

    AutoRoutine autoRoutine;
    
    @Override
    public void init() {
        super.init();
        
        robot.preload(
            new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE});
        
        robot.follower.setTelemetry(telemetry);
        
        robot.transfer.feedFromSpindexer();
        
        autoRoutine = robot.follower.autoBuilder(startingPose)
            .addRoutine(fireArtifacts(startingPose))
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
                .curveTo(pickUpPose1ControlPoint, pickUpPose1)
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
                .withTimeout(4)
            
            .addRoutine(fireArtifacts(pickUpPose1))
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
                .curveTo(pickUpPose2ControlPoint, pickUpPose2)
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
                .withTimeout(6)
            
            .lineTo(openGatePosition)
            
            .addRoutine(fireArtifacts(openGatePosition))
            
            .lineTo(prePickup3)
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(pickUpPose3, 4)
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            
            .addRoutine(fireArtifacts(pickUpPose3))
            
            .build();
    }
    
    public AutoRoutine fireArtifacts(Pose startingPose) {
        return robot.follower.autoBuilder(startingPose)
            .addAction(() -> robot.setState(Robot.State.REVVING))
            .lineTo(launchingPose).stop()
            .addAction(() -> robot.setState(Robot.State.LAUNCHING))
            .holdLastPath()
                .until(() -> robot.state == Robot.State.IDLE)
            .build();
    }
    
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
//
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(startingPose);
        autoRoutine.start();
    }
    
    boolean foundPattern = false;
    
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
        
        telemetry.addData("index", autoRoutine.getIndex());
        telemetry.addData("state", robot.state);
        telemetry.addData("pose", robot.follower.getCurrentPose());
        telemetry.update();
    }
}
