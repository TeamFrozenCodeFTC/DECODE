//package org.firstinspires.ftc.teamcode.testing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.AllianceColor;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.auto.Auto;
//
//@TeleOp
//public class TuneFlyWheel extends Auto {
//    Follower follower;
//
//    @Override
//    public void init() {
//        super.init();
//        follower = new Follower(hardwareMap);
//
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
//                                          telemetry);
//
//
//        robot.flywheel.setRPM(4000);
//        robot.flywheel.filteredVoltage = follower.getVoltage();
//
//        robot.follower.setCurrentPose(Robot.allianceColor.getHumanResetZone());
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad1.dpad_down) {
//            robot.flywheel.setRPM(robot.flywheel.getTargetRPM() - 50);
//        }
//        else if (gamepad1.dpad_up) {
//            robot.flywheel.setRPM(robot.flywheel.getTargetRPM() + 50);
//        }
//
//        telemetry.addData("distanceToGoal",
//                          Robot.allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition()));
//        telemetry.addData("current RPM", robot.flywheel.getRpm());
//        telemetry.addData("target RPM", robot.flywheel.getTargetRPM());
//        telemetry.update();
//
//        follower.update();
//
//        MotionState motion = follower.getMotionState();
//        double turn =
//            robot.follower.drivePowerController.computeHeadingCorrectionPower(getAngleToGoal(), motion);
//        robot.follower.drivetrain.followVector(motion.makeRobotRelative(new Vector(0,0)), turn);
//
//        robot.flywheel.update(0.01, follower.getVoltage());
//        robot.intakeRamp.outtake();
//        robot.resetSpindexer();
//    }
//
//    public double getAngleToGoal() {
//        return follower.getCurrentPose().getPosition().getAngleToLookAt(AllianceColor.BLUE.getGoalPosition());
//    }
//}
