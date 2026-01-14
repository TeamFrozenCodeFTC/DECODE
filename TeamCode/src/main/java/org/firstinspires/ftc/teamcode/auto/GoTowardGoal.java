package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class GoTowardGoal extends Auto {
    public Pose endPose = new Pose(0, 48, 0);
    
    @Override
    public void loop() {
        robot.update();
        robot.follower.holdPose(endPose);
    }
}
