package org.firstinspires.ftc.blackice.examples;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;
import org.firstinspires.ftc.blackice.util.geometry.Pose;

@Autonomous(group="Black-Ice Examples")
public class HoldPoses extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        waitForStart();
        
        Pose targetPose = new Pose(48, 0, 0);
        boolean goingToStart = false;
        
        while (opModeIsActive()) {
            follower.update();

            follower.holdPose(targetPose);

            if (follower.isStoppedAt(targetPose)) {
                if (goingToStart) {
                    targetPose = new Pose(48, 0, 0);
                } else {
                    targetPose = new Pose(0, 0, 0);
                }
                goingToStart = !goingToStart;
            }
            
            telemetry.addData("Current Pose",
                              follower.getCurrentPose().getPosition().getX());
            telemetry.addData("Target Pose", targetPose.getPosition().getX());
            telemetry.update();
        }
    }
}
