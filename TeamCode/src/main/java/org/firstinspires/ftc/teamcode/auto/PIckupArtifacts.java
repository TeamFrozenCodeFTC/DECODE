package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.steps.Step;
import org.firstinspires.ftc.teamcode.auto.steps.StepRunner;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;


@Autonomous
public class PIckupArtifacts extends Auto {
    public Pose endPose = new Pose(35, 0, 0);
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        robot.spindexer.rotateToSlot(0);
    }
    
    @Override
    public void loop() {
        robot.update();
        if (robot.spindexer.getNumberOfArtifacts() < 3) {
            robot.follower.holdPose(endPose);
            robot.state = Robot.State.INTAKING;
        }
        else {
            robot.follower.holdPose(new Pose(0,0,0));
        }
        
    }
}
