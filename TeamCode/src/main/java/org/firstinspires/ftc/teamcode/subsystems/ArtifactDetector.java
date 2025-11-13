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
    
//    public Spindexer.Artifact getDetectedArtifact() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//        final float[] hsvValues = new float[3];
//        Color.colorToHSV(colors.toColor(), hsvValues);
//
//        hue = hsvValues[0];
//        saturation = hsvValues[1];
//        distanceCm = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
//
//        // Ignore readings that are too far to be reliable
//        if (distanceCm > 8.5) {
//            return Spindexer.Artifact.NONE;
//        }
//
//        // Add minimum saturation to filter out gray/white noise
//        if (saturation < 0.25) {
//            return Spindexer.Artifact.NONE;
//        }
//
//        // Optional: Add a small hue offset to account for lighting differences
//        double adjustedHue = (hue + 360) % 360;
//
//        // Tighter color bounds and use overlapping zones for smoother transitions
//        boolean isPurple =
//            adjustedHue >= 175 && adjustedHue <= 255 &&
//                saturation >= 0.35 &&
//                colors.red > colors.green * 1.2;
//
//        boolean isGreen =
//            adjustedHue >= 100 && adjustedHue <= 170 &&
//                saturation >= 0.4 &&
//                colors.green > colors.red * 1.2;
//
//        if (isPurple && !isGreen) {
//            return Spindexer.Artifact.PURPLE;
//        } else if (isGreen && !isPurple) {
//            return Spindexer.Artifact.GREEN;
//        } else if (isGreen && isPurple) {
//            // Ambiguous detection, likely transitional lighting
//            return Spindexer.Artifact.UNKNOWN;
//        } else {
//            return Spindexer.Artifact.NONE;
//        }
//    }

}
