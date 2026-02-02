package org.firstinspires.ftc.teamcode.blackice.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;

@Autonomous
public class LocalizationTest extends OpMode {
    Follower follower;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        follower.setTelemetry(telemetry);
    }
    
    @Override
    public void loop() {
        follower.update();
        
        follower.telemetry.addData("position", follower.getPosition());
        follower.telemetry.addData("pose", follower.getCurrentPose());
        follower.telemetry.update();
    }
}
