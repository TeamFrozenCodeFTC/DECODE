package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class FarAuto3 extends Auto2 {
    Robot robot;
    
    public Pose startingPose = new Pose(56, 17.75/2, -90);
    public Pose launchingPose = new Pose(57, 17, -65);
    
    public Pose prePickupPose = new Pose(9, 30, -90);
    public Pose pickupPose = new Pose(9, 10, -90);
    
    public Pose prePickupPose1 = new Pose(43, 36, 180);
    public Pose pickupPose1 = new Pose(11, 36, 180);
    
    public Pose farPickUp3 = new Pose(17.75/2, 15.5, 180);
    
    public Pose endPose = new Pose(51, 21, -46);
    
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
            
            .lineTo(prePickupPose)
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(pickupPose)
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
                .withTimeout(5)
            
            .addRoutine(fireArtifacts(startingPose))
            
            .lineTo(prePickupPose1)
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(pickupPose1)
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
                .withTimeout(4)
            
            .addRoutine(fireArtifacts(startingPose))
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(farPickUp3)
                .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
                .withTimeout(5)
            
            .addRoutine(fireArtifacts(startingPose))
            
            .lineTo(endPose)
                .stop()
            .build();
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(startingPose);
        autoRoutine.start();
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
        
//        telemetry.addData("index", autoRoutine.getIndex());
//        telemetry.addData("state", robot.state);
//        telemetry.update();
    }
}
