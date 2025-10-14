package org.firstinspires.ftc.blackice.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;

@Autonomous(group="Black-Ice Examples")
public class ToPoints extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        PathRoutine path = follower.pathRoutineBuilder()
            .toPose(48, 0)
            .toPose(48+24, 24)
            .stop()
            .build();
        
        waitForStart();
        
        follower.follow(path);
        
        while (opModeIsActive()) {
            follower.update();
        }
    }
}
