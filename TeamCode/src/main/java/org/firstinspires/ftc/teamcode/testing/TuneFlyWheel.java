package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.Auto;

@TeleOp
public class TuneFlyWheel extends Auto {
    Flywheel2 flywheel2;
    
    @Override
    public void init() {
        super.init();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
                                          telemetry);


//        robot.flywheel.setRPM(4000);
//        robot.flywheel.filteredVoltage = robot.follower.getVoltage();
//        robot.follower.drivetrain.zeroPowerFloatMode();
//
        flywheel2 = new Flywheel2(hardwareMap);
//
//        robot.follower.setCurrentPose(Robot.allianceColor.getHumanResetZone());
    }

    @Override
    public void loop() {
        robot.follower.update();
        flywheel2.update(robot.follower.deltaTime, robot.follower.getVoltage());
        
        if (gamepad1.dpad_down) {
            flywheel2.setRPM(flywheel2.getTargetRPM() - 50);
        }
        else if (gamepad1.dpad_up) {
            flywheel2.setRPM(flywheel2.getTargetRPM() + 50);
        }
        
        telemetry.addData("current RPM", flywheel2.getRpm());
        telemetry.addData("target RPM", flywheel2.getTargetRPM());
        telemetry.update();
        
//
//        telemetry.addData("distanceToGoal",
//                          Robot.allianceColor.getGoalPosition().distanceTo(robot.follower.getCurrentPose().getPosition()));
//        telemetry.addData("current RPM", robot.flywheel.getRpm());
//        telemetry.addData("target RPM", robot.flywheel.getTargetRPM());
//        telemetry.update();
//
//        robot.follower.update();
//
//        double turn =
//            robot.follower.computeHeadingCorrectionPower(getAngleToGoal());
//        robot.follower.drivetrain.followVector(new Vector(0, 0), turn);
//
//        robot.flywheel.update(robot.follower.deltaTime, robot.follower.getVoltage());
//        robot.intakeRamp.intakeThrough();
//        robot.resetSpindexer();
    }

}
