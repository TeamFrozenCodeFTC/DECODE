package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArtifactDetector {
    public NormalizedColorSensor colorSensor;
    
    public ArtifactDetector(HardwareMap hardwareMap, String colorSensorName) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, colorSensorName);
        ((ColorSensor) colorSensor).enableLed(false);
    }
    
    public float hue;
    public float saturation;
    public double distanceCm;
    
    public void enable() {
        ((ColorSensor) colorSensor).enableLed(true);
    }
    
    public void disable() {
        ((ColorSensor) colorSensor).enableLed(false);
    }
    
    public Spindexer.Artifact getDetectedArtifact() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        
        hue = hsvValues[0];
        saturation = hsvValues[1];
        distanceCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        if (distanceCm > 8) {
            return Spindexer.Artifact.NONE;
        }
        else if (hue >= 180 && hue <= 240) {
            return Spindexer.Artifact.PURPLE;
        } else if (saturation > .6 && hue >= 120 && hue <= 170) {
            return Spindexer.Artifact.GREEN;
        }
        else {
            return Spindexer.Artifact.UNKNOWN;
        }
    }
}
