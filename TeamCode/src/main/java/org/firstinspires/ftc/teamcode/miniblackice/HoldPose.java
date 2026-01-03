package org.firstinspires.ftc.teamcode.miniblackice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.miniblackice.core.Follower;
import org.firstinspires.ftc.teamcode.miniblackice.geometry.Pose;

@Autonomous
public class HoldPose extends OpMode {
    Follower follower;
    
    Pose startingPose = new Pose(0, 0, 0);
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
    }
    
    @Override
    public void loop() {
        follower.update();
        
        follower.holdPose(startingPose);
    }
}
