package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class SlamIntoArtifacts extends Auto {
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
        robot.follower.holdPose(endPose);
        robot.state = Robot.State.STUFF_INTAKE;
    }
}
