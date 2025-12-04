package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Artifact;

public class ArtifactDetector {
    public NormalizedColorSensor colorSensor;
    
    public ArtifactDetector(HardwareMap hardwareMap, String colorSensorName) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, colorSensorName);
        ((ColorSensor) colorSensor).enableLed(false);
    }
    
    public float hue;
    
    public Artifact getDetectedArtifact() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        
        hue = hsvValues[0];

        if (hue >= 225 && hue <= 240) {
            return Artifact.PURPLE;
        } else if (hue >= 120 && hue < 160) {
            return Artifact.GREEN;
        }
        return Artifact.NONE;
    }
}
