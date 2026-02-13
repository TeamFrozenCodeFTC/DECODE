package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;
import org.firstinspires.ftc.teamcode.subsystems.spindexer.MotifPattern;
import org.firstinspires.ftc.teamcode.utils.DelayWrapper;

@Autonomous
public class FarAuto3 extends Auto2 {

    public Pose startingPose = new Pose(56, 17.75/2, -90);
    public Pose launchingPose = new Pose(57, 17, -65);
    
    // corner 3
    public Pose prePickupPose = new Pose(9, 30, -90);
    public Pose prePickupPoseb = new Pose(12, 15, -110);
    public Pose pickupPose = new Pose(9, 8, -90);
    
    // last 3
    public Pose prePickupPose1 = new Pose(43, 36, 180);
    public Pose pickupPose1 = new Pose(9, 36, 180);
    
    public Pose farPickUp3 = new Pose(17.75/2, 15.5, 180);
    
    public Pose endPose = new Pose(51, 21, -46);
    
    AutoRoutine autoRoutine;
    
    @Override
    public void init() {
        super.init();
        
        robot.preload(
            new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE});
        
        robot.follower.setTelemetry(telemetry);
    }
    
    @Override
    public void start() {
        super.start();
        
        robot.follower.setCurrentPose(startingPose);
 
        MotifPattern pattern = motifDetector.getMotifPattern();
        if (pattern != null) {
            Robot.motifPattern = pattern;
        }
        
        autoRoutine = robot.follower.autoBuilder(startingPose)
            .addRoutine(fireArtifacts(startingPose))
            
            .lineTo(prePickupPose1)
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(pickupPose1)
            .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            .withTimeout(4)
            
            .addRoutine(fireArtifacts(pickupPose1))
            
            .lineTo(prePickupPose).stop()
            .addAction(() -> robot.setState(Robot.State.INTAKING))
                .lineTo(prePickupPoseb)
                .until(() -> false)
                .withTimeout(0.5)
            .lineTo(pickupPose)
            .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            .withTimeout(5)
            
            .addRoutine(fireArtifacts(pickupPose))
            
            .addAction(() -> robot.setState(Robot.State.INTAKING))
            .lineTo(farPickUp3)
            .until(() -> robot.spindexer.getNumberOfArtifacts() == 3)
            .withTimeout(5)
            
            .addRoutine(fireArtifacts(farPickUp3))
            
            .lineTo(endPose)
            .stop()
            .build();
        
        autoRoutine.start();
    }
    
    public AutoRoutine fireArtifacts(Pose startingPose) {
        return robot.follower.autoBuilder(startingPose)
            .addAction(() -> robot.setState(Robot.State.REVVING))
            .lineTo(launchingPose).stop()
            .addAction(() -> robot.setState(Robot.State.LAUNCHING))
            .holdLastPath().until(new DelayWrapper(500, () -> robot.state == Robot.State.IDLE))
            .build();
    }
    
    @Override
    public void loop() {
        robot.update();
        autoRoutine.run();
        
        telemetry.addData("state", robot.state);
        telemetry.addData("firingTimer", robot.spindexer.firingTimer.seconds());
        telemetry.addData("numOfArtifacts", robot.spindexer.getNumberOfArtifacts());
        telemetry.addData("isUpToSpeed", robot.flywheel.isAtSpeed());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        motifDetector.stop();
    }
}
