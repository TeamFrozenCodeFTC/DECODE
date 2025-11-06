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
        
        waitForStart();
        
        while (opModeIsActive()) {
            PathRoutine path = follower.pathRoutineBuilder()
                .lineTo(48, 0)
                .stop()
                .lineTo(0,0)
                .stop()
                .build();
            
            follower.follow(path);
            
            while (opModeIsActive() && follower.isInProgress()) {
                follower.update();
            }
        }
    }
}
