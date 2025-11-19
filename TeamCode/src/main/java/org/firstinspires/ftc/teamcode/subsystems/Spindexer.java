package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
        
        public Artifact oppositeColor() {
            return this == Artifact.PURPLE ? Artifact.GREEN : Artifact.PURPLE;
        }
        
        public static final Artifact[] HUMAN_PLAYER_PATTERN = new Artifact[]
            {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
        public static final Artifact[] EMPTY_PATTERN = new Artifact[]
            {Artifact.NONE, Artifact.NONE, Artifact.NONE};
    }
    
    public Artifact[] slots = Artifact.EMPTY_PATTERN;

    public double currentSlotIndex = 2;
    
    public void resetSlots() {
        slots = new Artifact[]
            {Artifact.NONE, Artifact.NONE, Artifact.NONE};;
    }
    
    public Spindexer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        rightColorSensor = new ArtifactDetector(hardwareMap, "sensor_color");
        leftColorSensor = new ArtifactDetector(hardwareMap, "leftColorSensor");
    }

    public Artifact getDetectedArtifact() {
        Artifact detected = rightColorSensor.getDetectedArtifact();
        if (detected == Artifact.UNKNOWN || detected == Artifact.NONE) {
            Artifact secondDetected = leftColorSensor.getDetectedArtifact();
            if (secondDetected == Artifact.NONE && detected == Artifact.UNKNOWN) {
                return Artifact.UNKNOWN;
            }
        }
        return detected;
    }
    
    public static boolean hasDecimal(double value) {
        return value % 1 != 0;
    }
    
    public int getLeftIndex(double index) {
        return hasDecimal(index) ? (int)Math.floor(index) : (int)index - 1;
    }
    
    public int getRightIndex(double index) {
        return hasDecimal(index) ? (int)Math.ceil(index) : (int)index + 1;
    }
    
    boolean rotateLeft = false;
    boolean rotateRight = false;
    
    public boolean rotateToArtifact(Artifact artifact) {
        int leftIndex = getLeftIndex(currentSlotIndex);
        int rightIndex = getRightIndex(currentSlotIndex);
        
        int leftSlotIndex = rollIndex(leftIndex);
        int rightSlotIndex = rollIndex(rightIndex);
        
        boolean leftIsArtifact = slots[leftSlotIndex] == artifact;
        boolean rightIsArtifact = slots[rightSlotIndex] == artifact;
        
        if (leftIsArtifact && rightIsArtifact) {
            // continue previous rotation direction
        }
        else if (leftIsArtifact) {
            rotateLeft = true;
            rotateRight = false;
        }
        else if (rightIsArtifact) {
            rotateRight = true;
            rotateLeft = false;
        }
        else {
            rotateRight = false;
            rotateLeft = false;
        }
        
        if (rotateLeft) {
            rotateToSlot(leftIndex);
            slots[leftSlotIndex] = Spindexer.Artifact.NONE;
            return true;
        }
        else if (rotateRight) {
            rotateToSlot(rightIndex);
            slots[rightSlotIndex] = Spindexer.Artifact.NONE;
            return true;
        }
        return false;
    }
    
    public void forceRotateToArtifact(Artifact artifact) {
        if (!rotateToArtifact(artifact)) { // rotates to opposite color if not found
            rotateToArtifact(artifact.oppositeColor());
        }
    }
    
    public void rotateToSlot(double slotIndex) {
        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .483);
        
        currentSlotIndex = slotIndex;
    }
    

//    public void incomingArtifact(Artifact artifactType) {
//        int nextSlot = rollIndex(currentSlotIndex + 1);
//        int prevSlot = rollIndex(currentSlotIndex - 1);
//
//        if (slots[nextSlot].isNone()) {
//            rotateToSlot(nextSlot);
//        } else if (slots[prevSlot].isNone()) {
//            rotateToSlot(prevSlot);
//        }
//        else {
//            // partially rotate to way that gives both color options and lock artifacts
//            if (slots[nextSlot] == artifactType) {
//                // rotate away from it
//                partiallyRotate(prevSlot);
//            } else {
//                partiallyRotate(nextSlot);
//            }
//        }
//    }
    
    public static int rollIndex(int index) {
//        if (index < 0) {
//
//            // -1 -> 2
//            // -2 -> 1
//            // -3 -> 0
//            // -4 -> 2
//
//            // -1
//            return index + 3 + (-index % 3); // doesn't fully cover
//        } else if (index >= 3) {
//            return index % 3; // roll forward to first
//        } else {
//            return index;
//        }
        return index % 3;
    }
    
    public int getNumberOfArtifacts() {
        return (slots[0] != Artifact.NONE ? 1 : 0)
            + (slots[1] != Artifact.NONE ? 1 : 0)
            + (slots[2] != Artifact.NONE ? 1 : 0);
    }
}
