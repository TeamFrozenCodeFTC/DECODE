//package org.firstinspires.ftc.blackice.examples;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.blackice.core.follower.Follower;
//import org.firstinspires.ftc.blackice.util.geometry.Vector;
//
//
//
///**
// * Run a basic field-centric tele-
// * Sets motor powers based on the controls and brakes at zero power.
// * Will implement teleOp velocity constraints in the future.
// */
//@TeleOp
//public class FieldCentricTeleOp extends LinearOpMode {
//    public ViperSlide viperSlide;
//    public Intake intake;
//
//    @Override
//    public void runOpMode() {
//        Follower follower = new Follower(hardwareMap);
//
//        intake = new Intake(this.hardwareMap);
//        viperSlide = new ViperSlide(this.hardwareMap, intake);
//        follower.drivetrain.zeroPowerBrakeMode();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            follower.update();
//            telemetry.addData("heading", follower.getMotionState().heading);
//            telemetry.addData("field direction vector", new Vector(-gamepad1.left_stick_y,
//                -gamepad1.left_stick_x));
//            telemetry.addData("robot relative vector", follower.getMotionState().makeRobotRelative(new Vector(-gamepad1.left_stick_y,
//                -gamepad1.left_stick_x)));
//            telemetry.update();
//
//            follower.fieldCentricTeleOpDrive(
//                -gamepad1.left_stick_y * 0.4,
//                -gamepad1.left_stick_x * 0.4,
//                -gamepad1.right_stick_x * 0.4
//            );
//            intake.loopUpdate();
//            viperSlide.loopUpdate();
//            control();
//        }
//        //follower.waitUntilOpModeStop();
//
//    }
//
//    void control() {
//        linearSlide();
//        intakeExtender();
//        intakeArm();
//        intakeSweeper();
//        claw();
//    }
//
//    void claw() {
//        if (gamepad2.triangle) {
//            viperSlide.clawGrab();
//        }
//        if (gamepad2.cross) {
//            viperSlide.clawOut();
//        }
//    }
//
//    void linearSlide() {
//        if (gamepad2.dpad_up) {
//            viperSlide.upperChamberRaise();
//        }
//        else if (gamepad2.dpad_left) {
//            viperSlide.upperBasketRaise();
//        }
//        else if (gamepad2.dpad_right) {
//            viperSlide.raise(1700);
//        }
//        else if (gamepad2.dpad_down) {
//            viperSlide.lower();
//        }
//        else if (gamepad2.touchpad_finger_1) {
//            ElapsedTime timer = new ElapsedTime();
//            viperSlide.lower();
//            timer.reset();
//            while (timer.seconds() < 3) {
//                double progress = 1 - timer.seconds() / 3;
//                viperSlide.viperSlideMotor.setPower(progress);
//            }
//        }
//    }
//
//    void intakeExtender() {
//        if (gamepad2.right_stick_button) {
//            return;
//        }
//
//        double power = -gamepad2.left_stick_y;
//
//        if (power > 0) {
//            intake.intakeExtender.setTargetPosition(1244);
//        }
//        if (power < 0) {
//            intake.intakeExtender.setTargetPosition(0);
//        }
//
//        intake.intakeExtender.setPower(power);
//    }
//
//    void intakeArm() {
//        // Once a sample is acquired, this lifts and retracts the intake.
////        if (gamepad2.right_stick_button) {
////            intake.armUp();
////            intake.retract();
////            // go back
////        }
//        // Sweep Samples out of the side of the submersible.
//        if (gamepad2.left_stick_button) {
//            intake.armDown();
//            intake.spinSweeperBy(-1);
//        }
//        // Get Samples from submersible
//        else if (gamepad2.right_stick_y < -0.2) {
//            intake.armOut();
//            //sleep(500);
//            //sweeperRotator.getController().pwmDisable();
//        }
//        else if (gamepad2.right_stick_y > 0.2) {
////            intake.armIn();
//            intake.armIn();
//        }
////        else if (gamepad2.right_stick_button) {
////            intake.armIn();
////        }
//    }
//
//    void intakeSweeper() {
//        // Sweeper In
//        if (gamepad2.right_trigger > 0) {
//            intake.spinSweeperBy(gamepad2.right_trigger * 0.75);
//        }
//        else if (gamepad2.left_trigger > 0) {
//            intake.spinSweeperBy(-gamepad2.left_trigger * 0.65);
//        }
//        else {
//            intake.sweeper.setPower(0);
//        }
//        if (gamepad2.square) {
//            viperSlide.clawGrab();
//            viperSlide.dump();
//        }
//        else {
//            viperSlide.bucketDown();
//        }
//    }
//}
