package org.firstinspires.ftc.blackice.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Pose;

@TeleOp
@Config
public class HoldPosition extends LinearOpMode {
    public static double POSITION_DELTA = 0.5; // in per loop

    public static double HEADING_DELTA = 1.0;
    
    public static double MAX_SPEED = 30.0; // in/s
    public static double MAX_TURN_SPEED = 90.0; // deg/s
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        Pose target = new Pose(0, 0, 0);
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            follower.update();
            MotionState motionState = follower.getMotionState();

            if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0) {
                target =
                    new Pose(motionState.position.getX() - gamepad1.left_stick_y * POSITION_DELTA,
                             motionState.position.getY() - gamepad1.left_stick_x * POSITION_DELTA,
                             motionState.heading - gamepad1.right_stick_x * HEADING_DELTA);
                
            }

            follower.drivePowerController.holdPose(target, motionState);
            
            telemetry.addData("state", follower.getFollowingState());
            telemetry.update();
        }
    }
}
