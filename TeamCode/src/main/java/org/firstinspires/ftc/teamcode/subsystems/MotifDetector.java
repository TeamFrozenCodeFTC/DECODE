package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class MotifDetector {
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    
    private final HardwareMap hardwareMap;
    
    public MotifDetector(HardwareMap map) {
        hardwareMap = map;
    }

    public void start() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    public void stop() {
        visionPortal.stopStreaming();
        visionPortal.close();
    }
    
    public Artifact[] getMotifPattern() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        
        if (currentDetections.isEmpty()) {
            return null;
        }
        
        for (AprilTagDetection detection : currentDetections) {
            // all detections
            if (detection.id == 22) {
                return new Artifact[]{
                    Artifact.PURPLE,
                    Artifact.GREEN,
                    Artifact.PURPLE
                };
            } else if (detection.id == 23) {
                return new Artifact[]{
                    Artifact.PURPLE,
                    Artifact.PURPLE,
                    Artifact.GREEN
                };
            } else if (detection.id == 21) {
                return new Artifact[]{
                    Artifact.GREEN,
                    Artifact.PURPLE,
                    Artifact.PURPLE
                };
            } else {
                return null;
            }
        }
        return null;
    }
}
