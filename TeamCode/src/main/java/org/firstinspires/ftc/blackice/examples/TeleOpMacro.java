package org.firstinspires.ftc.blackice.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.blackice.util.actions.Action;
import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;
import org.firstinspires.ftc.teamcode.Haptics;

/**
 *
 */
@TeleOp
public class TeleOpMacro extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        
        Action rumbleAction = () -> gamepad1.rumble(Haptics.CONFIRM);
        follower.addDefaultPathBehavior(new PathBehavior()
            .onPause(rumbleAction)
            .onResume(rumbleAction)
            .setPauseTimeoutSeconds(8.0)
        );
        
        PathRoutine path = follower.pathRoutineBuilder()
//            .fromFuturePose()
            .toPose(0, 0)
            .stop()
            .build();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Goes to 0,0 when pressing dpad down
            if (gamepad1.dpadDownWasPressed()) {
                follower.follow(path);
                gamepad1.rumble(Haptics.CONFIRM);
            }
            
            follower.update();
            boolean isTryingToMove =
                Math.abs(gamepad1.left_stick_y) > 0.5 || Math.abs(gamepad1.left_stick_x) > 0.5;
            
            if (isTryingToMove) {
                follower.pause();
            }
            else if (gamepad1.x) {
                follower.resume();
            }

            if (!follower.isCommandingPower()) {
                follower.fieldCentricTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
                );
            }
            telemetry.addData("state", follower.getFollowingState());
            telemetry.update();
        }
    }
}
