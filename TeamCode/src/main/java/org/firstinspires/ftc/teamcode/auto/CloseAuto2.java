package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class CloseAuto2 extends OpMode {
    Robot robot;
    
    Pose startingPose = new Pose(33, 135, -90);
    Pose launchingPose = new Pose(50, 87, -48);

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
            .addAction(() -> robot.setState(Robot.State.REVVING))
            .lineTo(launchingPose)
                .linearHeadingInterpolation()
                .stop()
            .addAction(() -> robot.setState(Robot.State.LAUNCHING))
            //.holdLastPath().until(() -> robot.state == Robot.State.IDLE)
            .build();
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(startingPose);
        autoRoutine.start();
    }
    
    @Override
    public void loop() {
        robot.update();
        autoRoutine.run();
        
        telemetry.addData("index", autoRoutine.getIndex());
        telemetry.addData("state", robot.state);
        telemetry.addData("pose", robot.follower.getCurrentPose());
        telemetry.update();
    }
}
