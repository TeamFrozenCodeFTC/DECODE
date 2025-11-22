package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.Auto2;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;

@TeleOp
public class TuneFlyWheel extends Auto2 {
    MotifDetector aprilTag;
    
    @Override
    public void init() {
        super.init();
        aprilTag = new MotifDetector(hardwareMap);
        aprilTag.start();
        
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
                                          telemetry);
        
        
        robot.launcher.setRPM(4000);
    }
    
    @Override
    public void loop() {
        if (gamepad1.dpad_down) {
            robot.launcher.setRPM(robot.launcher.getTargetRPM() - 50);
        }
        else if (gamepad1.dpad_up) {
            robot.launcher.setRPM(robot.launcher.getTargetRPM() + 50);
        }
        
        telemetry.addData("current RPM", robot.launcher.getRpm());
        telemetry.addData("target RPM", robot.launcher.getTargetRPM());
        telemetry.update();
        
        robot.launcher.update(0.01);
    }
    
    @Override
    public void stop() {
        aprilTag.stop();
    }
}
