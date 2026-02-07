package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

@TeleOp
public class TuneFlyWheel extends Auto {
    @Override
    public void init() {
        super.init();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
                                          telemetry);


        robot.flywheel.setRPM(4000);
        robot.follower.drivetrain.zeroPowerFloatMode();

        robot.follower.setCurrentPose(Robot.allianceColor.getHumanResetZone());
        
        robot.resetSpindexer();
    }
    
    @Override
    public void start() {
        super.start();
        robot.follower.setCurrentPose(Robot.allianceColor.getHumanResetZone());
    }
    
    @Override
    public void loop() {
        robot.flywheel.readSensors();
        if (gamepad1.dpad_down) {
            robot.flywheel.setRPM(robot.flywheel.getTargetRPM() - 50);
        }
        else if (gamepad1.dpad_up) {
            robot.flywheel.setRPM(robot.flywheel.getTargetRPM() + 50);
        }

        telemetry.addData("distanceToGoal",
                          Robot.allianceColor.getGoalPosition().distanceTo(robot.follower.getCurrentPose().getPosition()));
        telemetry.addData("current RPM", robot.flywheel.getRPM());
        telemetry.addData("target RPM", robot.flywheel.getTargetRPM());
        telemetry.addData("targetHeading", getAngleToGoal());
        telemetry.addData("currentHeading", robot.follower.getHeading());
        telemetry.addData("pos", robot.follower.getPosition());
        telemetry.addData("goal Pos", Robot.allianceColor.getGoalPosition());
        
        telemetry.update();
        
        // -1.5
        
        robot.follower.update();

        double turn =
            robot.follower.computeHeadingCorrectionPower(getAngleToGoal());
        robot.follower.drivetrain.followVector(new Vector(0, 0), turn);

        robot.flywheel.update(robot.follower.deltaTime, robot.follower.getVoltage());
        robot.transfer.feedFromSpindexer();
    }
    
    public double getAngleToGoal() {
        return robot.follower.localizer.getPose()
            .getPosition()
            .getAngleToLookAt(Robot.allianceColor.getGoalPosition());
    }
}
