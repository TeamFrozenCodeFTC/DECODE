package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import android.graphics.Color;

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
        rightColor = new Poller<>(() -> computeHue(rightColorSensor), POLL_MS);
        leftColor = new Poller<>(() -> computeHue(leftColorSensor), POLL_MS);
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
    
    public float computeHue(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        
        return hsvValues[0];
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
