package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    Intake intake;
    Shooter shooter;
    Follower follower;
    
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        follower = new Follower(hardwareMap);
        follower.setCurrentPose(((Pose) blackboard.get("currentPose")));
    }
    
    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) {
            shooter.revUpShooterTo(1);
        }
        if (gamepad1.leftBumperWasPressed()) {
            shooter.stop();
        }
        
        intake.motor.setPower(gamepad1.right_stick_y);
        shooter.setPower(shooter.getCurrentPower() + gamepad1.right_stick_y * 0.005);
        
        telemetry.addData("velocity", "%.2f", shooter.getVelocity());
        telemetry.addData("power", "%.2f", shooter.getCurrentPower());
        telemetry.addData("currentPose", follower.getCurrentPose());
        telemetry.update();
        follower.update();
    }
}
