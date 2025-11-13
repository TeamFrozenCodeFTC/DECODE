package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;

@TeleOp
public class AprilTagAuto extends OpMode {
    AprilTag aprilTag;
    
    @Override
    public void init() {
        aprilTag = new AprilTag(hardwareMap);
        aprilTag.start();
        
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
                                          telemetry);
    }
    
    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            Spindexer.Artifact[] pattern = aprilTag.getPattern(); // replace Object with
            // actual type if you
            // know it
            telemetry.addData("pattern",
                              (pattern != null)
                                  ? (pattern.getClass().isArray() ? Arrays.deepToString((Object[]) pattern) : pattern)
                                  : "null"
            );
            telemetry.update();
        }

    }
    
    @Override
    public void stop() {
        aprilTag.stop();
    }
}
