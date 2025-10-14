package org.firstinspires.ftc.blackice.tuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

@Config
@TeleOp
public class NaturalDecelerationTuner extends LinearOpMode {
    public static double VELOCITY = 60;
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        waitForStart();
        
        follower.drivetrain.followVector(Vector.FORWARD.times(1), 0);
        
        while (follower.getMotionState().speed < VELOCITY) {
            follower.update();
        }
        follower.update();
        
        double maxVelocity = follower.getMotionState().speed;
        Vector distance = follower.getMotionState().position;
        
        follower.drivetrain.zeroPowerFloatMode();
        follower.drivetrain.followVector(Vector.ZERO, 0);
        
        while (follower.getMotionState().speed > 0.001) {
            follower.update();
        }
        follower.update();
        
        Vector finalDistance = follower.getMotionState().position;
        double displacement = finalDistance.minus(distance).computeMagnitude();
        
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("maxVelocity", maxVelocity);
        telemetry.addData("displacement", displacement);
        telemetry.addData("Your natural deceleration is",
                          maxVelocity * maxVelocity / (2 * displacement));
        telemetry.update();
        
        while (opModeIsActive()) idle();
    }
}
