//package org.firstinspires.ftc.blackice.examples.tests;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.blackice.core.control.QuadraticDampedPIDController;
//import org.firstinspires.ftc.blackice.core.follower.Follower;
//import org.firstinspires.ftc.blackice.util.geometry.Vector;
//
//
//@Config
//@Autonomous(group="Black-Ice Examples")
//public class HoldPointTest extends LinearOpMode {
//    public static double kP = 0.5;
//    public static double kLinearBraking = 0.07;
//    public static double kQuadraticFriction = 0.001;
//    public static double headingP = 2;
//
//    @Override
//    public void runOpMode() {
//        Follower follower = new Follower(hardwareMap);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            follower.update();
//
//            QuadraticDampedPIDController controller =
//                new QuadraticDampedPIDController(kP, kLinearBraking, kQuadraticFriction);
//
//            follower.drivetrain.followVector(
//                follower.getMotionState().makeRobotRelative(
//                    new Vector(0, 0).map(
//                        follower.getMotionState().position,
//                        (c1, c2) -> controller.run(c1, c2,
//                                                   follower.getMotionState().deltaTime)
//                )),
//                (0 - follower.getMotionState().heading) * headingP
//            );
//            telemetry.addData("position", follower.getMotionState().position);
//            telemetry.addData("field velocity", follower.getMotionState().velocity);
//            telemetry.addData("robot velocity", follower.getMotionState().robotRelativeVelocity);
//
//            telemetry.update();
//        }
//    }
//}
