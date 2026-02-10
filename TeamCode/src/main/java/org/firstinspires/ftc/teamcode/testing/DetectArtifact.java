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
    public void start() {
        boolean clear = robot.spindexer.isSpindexerClear();
        robot.spindexer.lastArtifactPresent = !clear;
        robot.spindexer.hadArtifactInitially = robot.spindexer.lastArtifactPresent;
        
        robot.spindexer.clearCycles = 0;
        robot.spindexer.firingTimer.resetAndStart();
        robot.spindexer.waitingForDrop = true;
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
        
        telemetry.addData("didArtifactJustDrop", robot.spindexer.didArtifactJustDrop());
        telemetry.addData("waitingForDrop", robot.spindexer.waitingForDrop);
        telemetry.addData("clear cycles", robot.spindexer.clearCycles);
        telemetry.addData("hadArtifactInitially", robot.spindexer.hadArtifactInitially);
        telemetry.addData("lastArtifactPresent", robot.spindexer.lastArtifactPresent);
        
        telemetry.addData("isSpindexerClear", robot.spindexer.isSpindexerClear());
        telemetry.addData("isArtifactInHandoffZone", robot.spindexer.isArtifactInHandoffZone());
        
        
        telemetry.addData("leftDistance",
                          robot.spindexer.distanceSensors.getLeftDistance());
        telemetry.addData("rightDistance",
                          robot.spindexer.distanceSensors.getRightDistance());
        
        telemetry.update();
    }
}
