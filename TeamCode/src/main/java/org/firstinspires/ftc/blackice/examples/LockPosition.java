package org.firstinspires.ftc.blackice.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

@TeleOp
public class LockPosition extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        Pose pose = new Pose(0, 0, 0);
        
        waitForStart();
        double lastTime = 0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            follower.update();
            MotionState motionState = follower.getMotionState();
            
            telemetry.addData("motionState dt", motionState.deltaTime);
            
            follower.holdPose(pose);
            
            double now = timer.seconds();
            telemetry.addData("seconds", (now - lastTime));
            telemetry.addData("Hz", 1/(now - lastTime));
            telemetry.update();
            
            lastTime = now;

//            Vector holdPower =
//                follower.drivePowerController.computeHoldPower(pose.getPosition(),
//                                                          motionState);
//
//
//            telemetry.addData("holdPower", holdPower);
//            telemetry.addData("positionDelta", pose.getPosition().minus(motionState.position));
//
//            telemetry.addData("local positionDelta",
//                              motionState.makeRobotRelative(pose.getPosition().minus(motionState.position)));
//
//            double turnPower = follower.drivePowerController.computeHeadingCorrectionPower(pose.getHeading(), motionState);
//
//            telemetry.addData("turnPower", turnPower);
//
//           follower.drivePowerController.followFieldVector(holdPower, turnPower,
//                                                             motionState);
//
//            telemetry.addData("currentPose", motionState.pose);
//            telemetry.addData("deltaTime", motionState.deltaTime);
//            telemetry.addData("Hz", 1/motionState.deltaTime);
//            telemetry.update();
        }
    }
}
