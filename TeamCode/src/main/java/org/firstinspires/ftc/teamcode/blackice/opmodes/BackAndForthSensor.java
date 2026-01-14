package org.firstinspires.ftc.teamcode.blackice.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@Autonomous
public class BackAndForthSensor extends OpMode {
    Follower follower;
    Robot robot;
    
    Pose targetPose = new Pose(48, 0, 0);
    Pose startingPose = new Pose(0, 0, 0);
    
    int state = 0;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap);
        
        robot.setState(Robot.State.CONTINUOUS_INTAKE);
    }
    
    @Override
    public void loop() {
        switch (state) {
            case 0:
                follower.holdPose(targetPose);
                if (follower.isStoppedAt(targetPose)) {
                    state++;
                }
                break;
            case 1:
                follower.holdPose(startingPose);
                if (follower.isStoppedAt(startingPose)) {
                    state--;
                }
                break;
        }
        
        telemetry.addData("position", follower.localizer.getPose());
        telemetry.addData("state", state);
        telemetry.update();
        
        follower.update();
        robot.update();
    }
}
