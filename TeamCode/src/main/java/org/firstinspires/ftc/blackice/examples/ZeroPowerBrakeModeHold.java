package org.firstinspires.ftc.blackice.examples;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.hardware.drivetrain.Mecanum;
import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

@TeleOp
@Config
public class ZeroPowerBrakeModeHold extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);

        follower.drivetrain.zeroPowerBrakeMode();
        
        waitForStart();
        
        while (opModeIsActive()) {
            for (DcMotorEx motor : ((Mecanum) follower.drivetrain).motors) {
                telemetry.addData("zero power behavior", motor.getZeroPowerBehavior());
            }
            telemetry.update();
        }
    }
}
