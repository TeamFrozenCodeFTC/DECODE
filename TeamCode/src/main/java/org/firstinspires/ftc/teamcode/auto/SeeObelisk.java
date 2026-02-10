package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;
import org.firstinspires.ftc.teamcode.subsystems.spindexer.MotifPattern;

import java.util.Arrays;

@TeleOp
public class SeeObelisk extends OpMode {
    MotifDetector MotifDetector;

    @Override
    public void init() {
        MotifDetector = new MotifDetector(hardwareMap);
        MotifDetector.start();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),
                                          telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            MotifPattern pattern = MotifDetector.getMotifPattern();
            telemetry.addData("pattern",
                              (pattern != null)
                                  ? (pattern.getClass().isArray() ?
                                  Arrays.deepToString((Object[]) pattern.getPattern()) : pattern)
                                  : "null"
            );
            telemetry.update();
        }

    }

    @Override
    public void stop() {
        MotifDetector.stop();
    }
}
