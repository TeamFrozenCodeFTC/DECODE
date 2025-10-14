package org.firstinspires.ftc.blackice.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.blackice.core.follower.Follower;

/**
 * Find the x, y, and heading of a position on the field. Useful for path making.
 * <p>
 * Start the robot in the corner of the field facing away from the side wall.
 */
@TeleOp(group = "Tuning")
public class MeasurePosition extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);

        waitForStart();

        follower.drivetrain.zeroPowerFloatMode();

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("x -> ", follower.getMotionState().position.getX());
            telemetry.addData("y  ^ ", follower.getMotionState().position.getY());
            telemetry.addData("robotVelocityX", follower.getMotionState().robotRelativeVelocity.getX());
            telemetry.addData("robotVelocityY", follower.getMotionState().robotRelativeVelocity.getY());
            telemetry.addData("fieldVelocityX", follower.getMotionState().velocity.getX());
            telemetry.addData("fieldVelocityY", follower.getMotionState().velocity.getY());
            telemetry.addData("heading", follower.getMotionState().heading);

            telemetry.update();
        }
    }
}
