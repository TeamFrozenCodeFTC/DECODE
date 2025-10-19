package org.firstinspires.ftc.blackice.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;
import org.firstinspires.ftc.blackice.util.Logger;

@Autonomous(group="Black-Ice Examples")
public class Decel extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        PathRoutine path = follower.pathRoutineBuilder()
            .lineTo(48 + 18, 0)
                .withBehavior(new PathBehavior()
                                  .decelerate(40) // 30
//                                  .setEndingVelocity(25)
//                                  .setEndingVelocityCruiseDistance(12)
                                  .continueMomentumAtEnd())
            .build();
        
        waitForStart();
        
        follower.follow(path);
        
        while (opModeIsActive()) {
            follower.update();
        }
    }
}
