package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class BackAndForth extends OpMode {
    Follower follower;
    
    Pose targetPose = new Pose(48, 0, 0);
    Pose startingPose = new Pose(0, 0, 0);
    
    AutoRoutine autoRoutine = new AutoRoutine(startingPose);
    
    int state = 0;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        autoRoutine.lineTo(new Pose(48, 0));
    }
    
    @Override
    public void loop() {
        switch (state) {
            case 0:
                if (follower.isWithinBraking(targetPose)) {
                    follower.holdPose(startingPose);
                    state++;
                    break;
                }
                follower.holdPose(targetPose);
                break;
            case 1:
                if (follower.isWithinBraking(startingPose)) {
                    follower.holdPose(targetPose);
                    state--;
                    break;
                }
                follower.holdPose(startingPose);
                break;
        }
        
        telemetry.addData("holdingPower",
                          follower.computeHoldPower(startingPose.getPosition()).computeMagnitude());
        telemetry.addData("position", follower.localizer.getPose());
        telemetry.addData("state", state);
        telemetry.update();
        
        follower.update();
    }
}
