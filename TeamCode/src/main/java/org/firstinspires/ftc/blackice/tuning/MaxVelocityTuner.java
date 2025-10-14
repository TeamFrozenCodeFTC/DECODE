package org.firstinspires.ftc.blackice.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

@TeleOp
public class MaxVelocityTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        waitForStart();
        
        follower.drivetrain.followVector(Vector.FORWARD.times(0.5), 0);
        
        sleep(2000);
        
        // velocity at end point with zero deceleration
        // vs velocity at end point with current velocity
        
        follower.update();
        follower.drivetrain.zeroPower();
        double maxVelocity = follower.getMotionState().speed;
        telemetry.addData("maxVelocity", maxVelocity);
        telemetry.addData("Your feedforward should be about", 1/maxVelocity);
        // 0.0158 for 63v
        telemetry.update();
        
        while (opModeIsActive()) idle();
    }
}
