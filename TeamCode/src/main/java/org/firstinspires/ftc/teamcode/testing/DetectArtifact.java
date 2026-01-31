package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous
public class DetectArtifact extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
    }

    @Override
    public void loop() {
        robot.spindexer.distanceSensors.update();
//        robot.spindexer.artifactDetector.update();
        
        telemetry.addData("detectedArtifact",
                                  robot.spindexer.getDetectedArtifact().toString());
        telemetry.addData("right hue",
                                  robot.spindexer.artifactDetector.getRightHue());
        telemetry.addData("left hue",
                                  robot.spindexer.artifactDetector.getLeftHue());
        
        telemetry.addData("isSpindexerClear", robot.spindexer.isSpindexerClear());
        telemetry.addData("isArtifactInHandoffZone", robot.spindexer.isArtifactInHandoffZone());
        
        
        telemetry.addData("leftDistance",
                          robot.spindexer.distanceSensors.getLeftDistance());
        telemetry.addData("rightDistance",
                          robot.spindexer.distanceSensors.getRightDistance());
        
        telemetry.update();
    }
}
