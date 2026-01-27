package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
        telemetry.addData("detectedArtifact",
                                  robot.spindexer.getDetectedArtifact().toString());
        telemetry.addData("right hue",
                                  robot.spindexer.artifactDetector.getRightHue());
        telemetry.addData("left hue",
                                  robot.spindexer.artifactDetector.getLeftHue());
        
        telemetry.addData("didArtifactJustEnterSpindexer",
                          robot.spindexer.didArtifactJustEnterSpindexer());
        telemetry.addData("didArtifactJustDrop", robot.spindexer.didArtifactJustDrop());
        
        telemetry.addData("leftDistance",
                          robot.spindexer.distanceSensors.getLeftDistance());
        telemetry.addData("rightDistance",
                          robot.spindexer.distanceSensors.getRightDistance());
        
        telemetry.update();
    }
}
