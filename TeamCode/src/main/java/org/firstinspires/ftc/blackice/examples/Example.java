package org.firstinspires.ftc.blackice.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;

@Autonomous(group="Black-Ice Examples")
public class Example extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap, new Pose(0, 0, 0));

        PathRoutine path = follower.pathRoutineBuilder()
            .lineTo(48, 0)
            .runAction(() -> {
                telemetry.addLine("Reached (48, 0)");
                telemetry.update();
            })
            .lineTo(48, 24)
                .withHeadingInterpolationTo(90)
            .stop()
            .build();
        
        waitForStart();
        
        follower.follow(path);

        while (opModeIsActive() && follower.isInProgress()) {
            follower.update();
        }
    }
}
