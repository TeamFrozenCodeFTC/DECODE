package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Spindexer {
    public ServoImplEx servo;
    public ColorRangeSensor colorSensor;
    
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

    public int currentSlotIndex = 0;
    
    public Spindexer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
    }
    
    public Artifact getDetectedArtifact() {
        if (colorSensor.getDistance(DistanceUnit.INCH) < 3.0) {
            return Artifact.NONE;
        }
        if (colorSensor.green() > 100) {
            return Artifact.GREEN;
        } else if (colorSensor.blue() > 100) {
            return Artifact.PURPLE;
        }
        return Artifact.NONE;
    }
    
    public void rotateToSlot(double slotIndex) {
        servo.setPosition(.409 * slotIndex + 0.093);
        currentSlotIndex = (int) slotIndex;
    }
    
    public void partiallyRotate(int slotIndex) {
        rotateToSlot(slotIndex + 0.5);
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
