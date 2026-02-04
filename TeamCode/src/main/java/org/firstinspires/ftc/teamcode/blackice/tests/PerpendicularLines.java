package org.firstinspires.ftc.teamcode.blackice.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.commands.AutoRoutine;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class PerpendicularLines extends OpMode {
    Follower follower;
    
    Pose targetPose = new Pose(48, 0, 0);
    Pose startingPose = new Pose(0, 0, 0);
    
    AutoRoutine autoRoutine;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        follower.setTelemetry(telemetry);
        
        autoRoutine = follower.autoBuilder(startingPose)
            .lineTo(targetPose)
            .lineTo(new Pose(48, 24,0))
            //.stop()
            .until((f, p) -> false)
            .build();
    }
    
    @Override
    public void start() {
        autoRoutine.start();
    }
    
    @Override
    public void loop() {
        follower.update();
        
        autoRoutine.run();
    }
}
