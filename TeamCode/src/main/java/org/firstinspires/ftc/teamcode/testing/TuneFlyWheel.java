package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.auto.Auto2;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;

@TeleOp
public class TuneFlyWheel extends Auto2 {
    Follower follower;

    @Override
    public void init() {
        super.init();

        follower = new Follower(hardwareMap);
        
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
                                          telemetry);
        
        
        robot.launcher.setRPM(4000);
        robot.launcher.filteredVoltage = follower.getVoltage();
        
        // 3500, 100
        // 3300, 62
        // 400, 117.5
        // 4250, 125.75
        // 4500, 136
        // 4800, 155.5
        
        robot.follower.setCurrentPose(robot.allianceColor.getHumanPlayerZone());
    }
    
    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            robot.launcher.setRPM(robot.launcher.getTargetRPM() - 50);
        }
        else if (gamepad1.dpad_up) {
            robot.launcher.setRPM(robot.launcher.getTargetRPM() + 50);
        }
        
        telemetry.addData("distanceTOGaol",
                          robot.allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition()));
        telemetry.addData("current RPM", robot.launcher.getRpm());
        telemetry.addData("target RPM", robot.launcher.getTargetRPM());
        telemetry.update();
        
        robot.follower.fieldCentricTeleOpDrive(
            gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            -gamepad1.right_stick_x
        );
        
        if (gamepad1.circle) {
            follower.lockHeadingAt(getAngleToGoal());
        }
        
        follower.update();
        
        robot.launcher.update(0.01, follower.getVoltage());
        robot.intakeRamp.outtake();
        robot.resetSpindexer();
    }
    
    
    public double getAngleToGoal() {
        return follower.getCurrentPose().getPosition().getAngleToLookAt(AllianceColor.BLUE.getGoalPosition());
    }
}
