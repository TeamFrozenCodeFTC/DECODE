package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.spindexer.MotifPattern;

@Autonomous
public class FarAuto2 extends Auto2 {
    Robot robot;
    
    Pose startingPose = new Pose(60, 17.75/2, -90);
    Pose launchingPose = new Pose(60, 18, -67);
    Pose farPickUp = new Pose(12.5, 19.5, -135);
    Pose farPickUpEnd = new Pose(16.5/2, 17.75/2, -90);
    
    Pose farPickUp2 = new Pose(17.75/2, 16.5/2+1, 180);
    
    Pose farPickUp3 = new Pose(17.75/2, 15.5, 180);
    
    Pose endPose = new Pose(39, 15, 0);
    
    AutoRoutine autoRoutine;
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.isAuto = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.preload(
            new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE});
        
        robot.follower.setTelemetry(telemetry);
        
        autoRoutine = robot.follower.autoBuilder(startingPose)
            .addRoutine(fireArtifacts(startingPose))
            
            .lineTo(farPickUp)
                .stop()
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(farPickUpEnd)
                .linearHeadingInterpolation()
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            
            .addRoutine(fireArtifacts(farPickUpEnd))
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(farPickUp2)
                .linearHeadingInterpolation()
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            
            .addRoutine(fireArtifacts(farPickUp2))
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
                .lineTo(farPickUp3)
                .linearHeadingInterpolation()
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            
            .addRoutine(fireArtifacts(farPickUp3))
            
            .lineTo(endPose)
                .stop()
            .build();
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(startingPose);
        autoRoutine.start();
        
        MotifPattern pattern = motifDetector.getMotifPattern();
        if (pattern != null) {
            Robot.motifPattern = pattern;
        }
    }
    
    public AutoRoutine fireArtifacts(Pose startingPose) {
        return robot.follower.autoBuilder(startingPose)
            .addAction(() -> robot.setState(Robot.State.REVVING))
            .lineTo(launchingPose).stop()
            .addAction(() -> robot.setState(Robot.State.LAUNCHING))
            .holdLastPath().until(() -> robot.state == Robot.State.IDLE)
            .build();
    }

    @Override
    public void loop() {
        robot.update();
        autoRoutine.run();
    }
}
