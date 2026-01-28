package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.Artifact;

public class ArtifactDetector {
    private static final int POLL_MS = 0;
    
    public Poller<Float> leftColor;
    public Poller<Float> rightColor;
    
    private float leftHue;
    private float rightHue;
    
    private Artifact detectedArtifact;

    public ArtifactDetector(HardwareMap hardwareMap) {
        NormalizedColorSensor leftColorSensor =
            hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        NormalizedColorSensor rightColorSensor =
                hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
        rightColor = new Poller<>(() -> computeHue(rightColorSensor), POLL_MS, 0);
        leftColor = new Poller<>(() -> computeHue(leftColorSensor), POLL_MS, POLL_MS / 2);
    }
    
    public void update() {
        rightHue = rightColor.poll();
        Artifact rightArtifact = detectFromHue(rightHue);
        
        if (rightArtifact.isArtifact()) {
            detectedArtifact = rightArtifact;
            return;
        }
        
        leftHue = leftColor.poll();
        Artifact leftArtifact = detectFromHue(leftHue);
        
        if (leftArtifact.isArtifact()) {
            detectedArtifact = leftArtifact;
        }
        else {
            detectedArtifact = Artifact.NONE;
        }
    }
    
    public Artifact getDetectedArtifact() {
        return detectedArtifact;
    }
    
    /** Computes hue without calculating HSV */
    private float computeHue(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        
        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;
        
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;
        
        if (delta == 0) return 0;
        
        float hue;
        if (max == r) hue = 60 * (((g - b) / delta) % 6);
        else if (max == g) hue = 60 * (((b - r) / delta) + 2);
        else hue = 60 * (((r - g) / delta) + 4);
        
        if (hue < 0) hue += 360;
        return hue;
    }
    
    private Artifact detectFromHue(float hue) {
        if (hue >= 225 && hue <= 240) return Artifact.PURPLE;
        if (hue >= 120 && hue < 160) return Artifact.GREEN;
        return Artifact.NONE;
    }
    
    public double getLeftHue() {
        return leftHue;
    }
    
    public double getRightHue() {
        return rightHue;
    }
}
