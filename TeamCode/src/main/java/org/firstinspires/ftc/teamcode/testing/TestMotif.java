package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;
import org.firstinspires.ftc.teamcode.subsystems.spindexer.MotifPattern;

@TeleOp
public class TestMotif extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MotifDetector d = new MotifDetector(hardwareMap);
        d.start();
        waitForStart();
        while (opModeIsActive()) {
            MotifPattern p = d.getMotifPattern();
            telemetry.addData("Motif", p == null ? "null" : p.toString());
            telemetry.update();
        }
        d.stop();
    }
}
