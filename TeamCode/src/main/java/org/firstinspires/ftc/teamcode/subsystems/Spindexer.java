package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spindexer {
    public ServoImplEx servo;
    public ArtifactDetector rightColorSensor;
    public ArtifactDetector leftColorSensor;
    
    public enum Artifact {
        NONE,
        UNKNOWN,
        PURPLE,
        GREEN;
        
        public boolean isArtifact() {
            return this != NONE;
        }
        
        public boolean isNone() {
            return this == NONE;
        }
    }
    
    public Artifact[] slots = new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE};

    public int currentSlotIndex = 2;
    
    public void resetSlots() {
        slots = new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE};
    }
    
    public void turnArtifactsBack() {
        rotateToSlot(0);
    }
    
    public Spindexer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        rightColorSensor = new ArtifactDetector(hardwareMap, "sensor_color");
        leftColorSensor = new ArtifactDetector(hardwareMap, "leftColorSensor");
    }
    
    public void enableSensors() {
        rightColorSensor.enable();
        leftColorSensor.enable();
    }
    
    public void disableSensors() {
        rightColorSensor.disable();
        leftColorSensor.disable();
    }
    
    
    
    public Artifact getDetectedArtifact() {
        Artifact detected = rightColorSensor.getDetectedArtifact();
        if (detected == Artifact.UNKNOWN || detected == Artifact.NONE) {
            Artifact secondDetected = leftColorSensor.getDetectedArtifact();
            if (secondDetected == Artifact.NONE) {
                return Artifact.UNKNOWN;
            }
        }
        return detected;
    }
    
    public void rotateToSlot(double slotIndex) {
        
        // 1800 / 60 = 30 slots
        
        // 60 / 1800 = .0333 per slot
        
        //servo.setPosition(slotIndex * ((double) 60 / 1800));
        

        // -.5 -> 0
        // 0 -> .2
        // 1 -> .6
        // 2 -> 1
        servo.setPosition(slotIndex * 0.4 + 0.2);
        currentSlotIndex = (int) slotIndex;
    }
    
    public void partiallyRotate(double slotIndex) {
        servo.setPosition((slotIndex + .5) * 0.4 + 0.2);
    }

    public void incomingArtifact(Artifact artifactType) {
        int nextSlot = rollIndex(currentSlotIndex + 1);
        int prevSlot = rollIndex(currentSlotIndex - 1);
        
        if (slots[nextSlot].isNone()) {
            rotateToSlot(nextSlot);
        } else if (slots[prevSlot].isNone()) {
            rotateToSlot(prevSlot);
        }
        else {
            // partially rotate to way that gives both color options and lock artifacts
            if (slots[nextSlot] == artifactType) {
                // rotate away from it
                partiallyRotate(prevSlot);
            } else {
                partiallyRotate(nextSlot);
            }
        }
    }
    
    public static int rollIndex(int index) {
        if (index < 0) {
            return 2; // roll back to last
        } else if (index >= 3) {
            return 0; // roll forward to first
        } else {
            return index;
        }
    }
    
    public boolean isAtMaxCapacity() {
        return slots[0] != Artifact.NONE && slots[1] != Artifact.NONE && slots[2] != Artifact.NONE;
    }
    
    public int getNumberOfArtifacts() {
        return (slots[0] != Artifact.NONE ? 1 : 0)
            + (slots[1] != Artifact.NONE ? 1 : 0)
            + (slots[2] != Artifact.NONE ? 1 : 0);
    }
    
    public Artifact getCurrentSlotIndex() {
        return slots[currentSlotIndex];
    }
}
