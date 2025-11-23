package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Artifact;

public class Spindexer {
    public ServoImplEx servo;
    public ArtifactDetector rightColorSensor;
    public ArtifactDetector leftColorSensor;
    
    public Artifact[] slots = Artifact.getEmptyPattern();

    public double currentSlotIndex = 2;
    
    public void resetSlots() {
        slots = new Artifact[]
            {Artifact.NONE, Artifact.NONE, Artifact.NONE};
    }
    
    public Spindexer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        rightColorSensor = new ArtifactDetector(hardwareMap, "sensor_color");
        leftColorSensor = new ArtifactDetector(hardwareMap, "leftColorSensor");
    }

    public Artifact getDetectedArtifact() {
        Artifact detected = rightColorSensor.getDetectedArtifact();
        if (detected == Artifact.NONE) {
           return leftColorSensor.getDetectedArtifact();
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
    
    public boolean rotateLeft = false;
    public boolean rotateRight = false;
    
    public boolean rotateToArtifact(Artifact artifact) {
        int leftIndex = getLeftIndex(currentSlotIndex);
        int rightIndex = getRightIndex(currentSlotIndex);
        
        int leftSlotIndex = rollIndex(leftIndex);
        int rightSlotIndex = rollIndex(rightIndex);
        
        boolean leftIsArtifact = slots[leftSlotIndex] == artifact;
        boolean rightIsArtifact = slots[rightSlotIndex] == artifact;
        
        if (leftIsArtifact && rightIsArtifact && (rotateRight || rotateLeft)) {
            // continue previous rotation direction
            //
        }
        else if (leftIsArtifact) {
            rotateLeft = true;
            rotateRight = false;
        }
        else if (rightIsArtifact) {
            rotateRight = true;
            rotateLeft = false;
        }
//        else {
//            return false;
////            rotateRight = false;
////            rotateLeft = false;
//        }
        
        if (rotateLeft) {
            rotateToSlot(leftIndex);
            slots[leftSlotIndex] = Artifact.NONE;
            return true;
        }
        else if (rotateRight) {
            rotateToSlot(rightIndex);
            slots[rightSlotIndex] = Artifact.NONE;
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
    
    public static int rollIndex(int index) {
        return Math.floorMod(index, 3);
    }
    
    public int getNumberOfArtifacts() {
        return (slots[0].isArtifact() ? 1 : 0)
            + (slots[1].isArtifact() ? 1 : 0)
            + (slots[2].isArtifact() ? 1 : 0);
    }
}
