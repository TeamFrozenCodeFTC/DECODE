package org.firstinspires.ftc.blackice.examples.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

@TeleOp(group="Black-Ice Examples")
public class FollowVectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        waitForStart();
        
        follower.drivetrain.followVector(new Vector(1, 0), 0);
        
        while (opModeIsActive()) idle();
    }
}
