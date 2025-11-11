package org.firstinspires.ftc.blackice.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.util.geometry.Pose;

@TeleOp
public class LockPosOpMode extends OpMode {
    Follower follower;
    
    Pose pose = new Pose(0, 0, 0);
    
    @Override
    public void loop() {
        follower.update();
        MotionState motionState = follower.getMotionState();
        
        telemetry.addData("motionState dt", motionState.deltaTime);
        
        follower.holdPose(pose);
        
        double now = timer.seconds();
        telemetry.addData("seconds", (now - lastTime));
        telemetry.addData("Hz", 1 / (now - lastTime));
        telemetry.update();
        
        lastTime = now;
    }
    
    double lastTime = 0;
    ElapsedTime timer = new ElapsedTime();
    
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
    }
}
