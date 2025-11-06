package org.firstinspires.ftc.blackice.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

@Autonomous(group="Black-Ice Examples")
public class FollowVectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        waitForStart();
        
        follower.drivetrain.followVector(new Vector(1, 0), 0);
        
        while (opModeIsActive()) idle();
    }
}
