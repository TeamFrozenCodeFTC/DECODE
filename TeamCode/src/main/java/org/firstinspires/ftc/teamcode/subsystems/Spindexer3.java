package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;

import java.util.Arrays;
import java.util.Collections;

public class Spindexer3 {
    public ServoImplEx servo;
    public ArtifactDetector rightColorSensor;
    public ArtifactDetector leftColorSensor;
    
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    
    public Artifact[] artifacts = Artifact.getEmptyPattern();

    public double currentSlotIndex = 0;
    
    public void resetSlots() {
        artifacts = new Artifact[]
            {Artifact.NONE, Artifact.NONE, Artifact.NONE};
        rotateLeft = false;
        rotateRight = false;
    }

    public int shiftLeft(double index, int steps) {
        int result = (int)Math.ceil(index);
        return result - steps;
    }
    
    public int shiftRight(double index, int steps) {
        int result = (int)Math.floor(index);
        return result + steps;
    }
    
    public int count(Artifact artifact) {
        return Collections.frequency(Arrays.asList(artifacts), artifact);
    }
    
    public Spindexer3(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        rightColorSensor = new ArtifactDetector(hardwareMap, "sensor_color");
        leftColorSensor = new ArtifactDetector(hardwareMap, "leftColorSensor");
        
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class,
                                              "rightDistanceSensor");
    }

    public Artifact getDetectedArtifact() {
        Artifact detected = rightColorSensor.getDetectedArtifact();
        if (detected == Artifact.NONE) {
           return leftColorSensor.getDetectedArtifact();
        }
        return detected;
    }
    
    public boolean artifactIsInSpindexer() {
        return leftDistanceSensor.getDistance(DistanceUnit.INCH) < 3
            || rightDistanceSensor.getDistance(DistanceUnit.INCH) < 3;
    }
    
    public enum Direction {
        LEFT,
        RIGHT
    }
    
    public Direction lastDirection = null;
    
    public int findBestRotationToArtifact(Artifact artifact) {
        int leftIndex = shiftLeft(currentSlotIndex, 1);
        int rightIndex = shiftRight(currentSlotIndex, 1);
        
        int leftSlotIndex = rollIndex(leftIndex);
        int rightSlotIndex = rollIndex(rightIndex);
        
        boolean leftIsArtifact = artifacts[leftSlotIndex] == artifact;
        boolean rightIsArtifact = artifacts[rightSlotIndex] == artifact;
        
        if (!(leftIsArtifact || rightIsArtifact)) {
            return lastDirection == Direction.LEFT ? leftIndex : rightIndex;
        }
        
        if (leftIsArtifact && rightIsArtifact && lastDirection != null) {
            // keep direction
        } else if (leftIsArtifact) {
            lastDirection = Direction.LEFT;
        } else {
            lastDirection = Direction.RIGHT;
        }
        
        return lastDirection == Direction.LEFT ? leftIndex : rightIndex;
    }
    
    // +1 is to the right, clockwise
    public boolean intakeArtifact(Artifact artifact) {
        int count = getNumberOfArtifacts();
        artifacts[count] = artifact;
        
        switch (count) {
            case 0:
                rotateToSlot(1);
                break;
            case 1:
                rotateToSlot(2);
                break;
            case 2:
                if (artifacts[1] == artifact) {
                    rotateToSlot(2.5);
                }
                else {
                    rotateToSlot(1.5);
                }
                return true;
        }
        return false;
    }
    
    public static boolean hasDecimal(double value) {
        return value % 1 != 0;
    }
    
    public boolean rotateLeft = false;
    public boolean rotateRight = false;
    
    public Integer chooseRotationTarget(Artifact artifact) {
        int leftIndex = shiftLeft(currentSlotIndex, 1);
        int rightIndex = shiftRight(currentSlotIndex, 1);
        
        int leftSlotIndex = rollIndex(leftIndex);
        int rightSlotIndex = rollIndex(rightIndex);
        
        boolean leftIsArtifact = artifacts[leftSlotIndex] == artifact;
        boolean rightIsArtifact = artifacts[rightSlotIndex] == artifact;
        
        if (!(leftIsArtifact || rightIsArtifact)) {
            rotateLeft = rotateRight = false;
            return null;
        }
        
        if (leftIsArtifact && rightIsArtifact && (rotateLeft || rotateRight)) {
            // keep previous direction
        } else if (leftIsArtifact) { // left is negative so gets closer to reversing
            // spindexer from intaking positive
            rotateLeft = true;
            rotateRight = false;
        } else {
            rotateRight = true;
            rotateLeft = false;
        }
        
        return rotateLeft ? leftIndex : rightIndex;
    }
    
    @Deprecated
    public boolean _rotateToArtifact(Artifact artifact) {
        Integer target = chooseRotationTarget(artifact);
        if (target == null) return false;
        
        int slotIndex = rollIndex(target);
        rotateToSlot(target);
        artifacts[slotIndex] = Artifact.NONE;
        return true;
    }
    
    public boolean rotateToArtifact(Artifact artifact) {
        Integer target = chooseRotationTarget(artifact);
        if (target == null) return false;
        
        rotateToSlot(target);
        return true;
    }
    
    public void rotateToSlot(double slotIndex) {
//        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .485);
        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .505);
        
        currentSlotIndex = slotIndex;
    }
    
    public int getNumberOfArtifacts() {
        return (artifacts[0].isArtifact() ? 1 : 0)
            + (artifacts[1].isArtifact() ? 1 : 0)
            + (artifacts[2].isArtifact() ? 1 : 0);
    }
    
    public static int rollIndex(int index) {
        return Math.floorMod(index, 3);
    }
}
