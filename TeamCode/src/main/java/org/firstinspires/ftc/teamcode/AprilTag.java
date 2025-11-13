package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private final HardwareMap hardwareMap;

    public AprilTag(HardwareMap map) {
        hardwareMap = map;
    }

    // start the april tag library
    public void start() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        
    }

    // stop the april tag library
    public void stop() {
        visionPortal.stopStreaming();
        visionPortal.close();
    }

    public Spindexer.Artifact[] getPattern(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.isEmpty()) {
            return null;
        }

        AprilTagDetection detection= currentDetections.get(0);
        if (detection.id==22){
             return new Spindexer.Artifact[] {
                 Spindexer.Artifact.PURPLE,
                     Spindexer.Artifact.GREEN,
                     Spindexer.Artifact.PURPLE
             };
        } else if (detection.id==23){
            return new Spindexer.Artifact[] {
                    Spindexer.Artifact.PURPLE,
                    Spindexer.Artifact.PURPLE,
                    Spindexer.Artifact.GREEN
            };
        } else if (detection.id ==21) {
            return new Spindexer.Artifact[]{
                    Spindexer.Artifact.GREEN,
                    Spindexer.Artifact.PURPLE,
                    Spindexer.Artifact.PURPLE
            };
        } else {
            return null;
        }
    }
}
