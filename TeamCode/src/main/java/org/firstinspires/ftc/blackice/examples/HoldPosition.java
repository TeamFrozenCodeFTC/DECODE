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
    public static double POSITION_DELTA = 2; // in per loop
    public static double HEADING_DELTA = 0.2;
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        follower.drivetrain.zeroPowerBrakeMode();
        
        Pose target = new Pose(0, 0, 0);
        
        waitForStart();
        
        boolean isHolding = false;
        boolean decelerating = false;
        
        while (opModeIsActive()) {
            
            follower.update();
            MotionState motionState = follower.getMotionState();

            boolean isInputing =
                Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0;
            
            if (!isInputing && !isHolding && !decelerating) {
                decelerating = true;
            }
            
            if (motionState.speed < 1 && decelerating) {
                decelerating = false;
                isHolding = true;
                target = motionState.pose;
            }
            
            if (isInputing || decelerating) {
                isHolding = false;
                follower.fieldCentricTeleOpDrive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
                );
            }
            else {
                follower.holdPose(target);
            }
            
            telemetry.addData("state", follower.getFollowingState());
            telemetry.update();
        }
    }
}
