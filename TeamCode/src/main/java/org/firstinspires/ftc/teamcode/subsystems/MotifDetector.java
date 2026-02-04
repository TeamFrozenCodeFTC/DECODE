package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Artifact;

import java.util.List;

public class MotifDetector {
    private Limelight3A limelight;
    private final HardwareMap hardwareMap;

    public MotifDetector(HardwareMap map) {
        hardwareMap = map;
    }

    public void start() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(2);
        limelight.start();
    }

    public void stop() {
        limelight.shutdown();
    }

    public Artifact[] getMotifPattern() {
        int tagid;
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tagid = fr.getFiducialId();
                // all detections
                if (tagid == 22) {
                    return new Artifact[]{
                            Artifact.PURPLE,
                            Artifact.GREEN,
                            Artifact.PURPLE
                    };
                } else if (tagid == 23) {
                    return new Artifact[]{
                            Artifact.PURPLE,
                            Artifact.PURPLE,
                            Artifact.GREEN
                    };
                } else if (tagid == 21) {
                    return new Artifact[]{
                            Artifact.GREEN,
                            Artifact.PURPLE,
                            Artifact.PURPLE
                    };
                }
            }
        }
        return null;
    }
}
