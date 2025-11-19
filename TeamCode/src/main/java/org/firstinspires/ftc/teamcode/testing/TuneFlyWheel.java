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
    }
    
    @Override
    public void loop() {
        robot.launcher.updateCoefficients();
        robot.launcher.setRPM(4000);
        
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
