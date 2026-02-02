package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.Artifact;

public class ArtifactDetector {
    public Poller<Float> leftColor;
    public Poller<Float> rightColor;
    
    private float leftHue;
    private float rightHue;
    
    NormalizedColorSensor leftColorSensor;
    NormalizedColorSensor rightColorSensor;
    
    private Artifact detectedArtifact;

    public ArtifactDetector(HardwareMap hardwareMap) {
        leftColorSensor =
            hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        rightColorSensor =
                hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
    }
    
    public void update() {
        rightHue = computeHue(rightColorSensor);
        Artifact rightArtifact = detectFromHue(rightHue);
        
        if (rightArtifact.isArtifact()) {
            detectedArtifact = rightArtifact;
            return;
        }
        
        leftHue = computeHue(leftColorSensor);
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
