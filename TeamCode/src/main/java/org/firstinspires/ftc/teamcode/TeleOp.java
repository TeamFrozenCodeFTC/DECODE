package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    Robot robot;
    AllianceColor allianceColor;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        if (blackboard.get("currentPose") != null) {
            robot.follower.setCurrentPose(((Pose) blackboard.get("currentPose")));
        }
        if (blackboard.get("allianceColor") != null) {
            allianceColor = (AllianceColor) blackboard.get("allianceColor");
        } else {
            allianceColor = AllianceColor.BLUE;
        }
        
        telemetry.update();
    }
    
    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) {
            robot.shooter.setRPM(3500);
        }
        if (gamepad1.leftBumperWasPressed()) {
            robot.shooter.stop();
        }
        
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.intake();
        }
        if (gamepad1.left_trigger > 0.1) {
            robot.intake.outtake();
        }
        if (gamepad1.triangle) {
            robot.intake.stop();
        }
        
        if (gamepad1.dpad_down) {
            robot.shooter.setRPM(robot.shooter.getTargetRPM() - 50);
        }
        if (gamepad1.dpad_up) {
            robot.shooter.setRPM(robot.shooter.getTargetRPM() + 50);
        }
        
        telemetry.addData("current rpm", "%.2f", robot.shooter.getRpm());
        telemetry.addData("target rpm", "%.2f", robot.shooter.getTargetRPM());
        
        telemetry.addData("currentPose", robot.follower.getCurrentPose());
        telemetry.addData("voltage", robot.follower.getVoltage());
        
        telemetry.update();
        
        robot.follower.fieldCentricTeleOpDrive(
            gamepad1.left_stick_x,
            -gamepad1.left_stick_y,
            -gamepad1.right_stick_x
        );
        robot.update();
    }
}
