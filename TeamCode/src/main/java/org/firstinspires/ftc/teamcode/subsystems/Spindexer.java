package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;

public class Spindexer {
    public ServoImplEx servo;
    public ArtifactDetector rightColorSensor;
    public ArtifactDetector leftColorSensor;
    
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    
    public Artifact[] slots = Artifact.getEmptyPattern();
    
    // Rotates spindexer in opposite direction to choose between two different artifacts
    public boolean reversedSpindexerIntake = false;
    public int dropAllIndex = 0;
    
    public double currentSlotIndex = 2;
    
    public void resetSlots() {
        slots = new Artifact[]
            {Artifact.NONE, Artifact.NONE, Artifact.NONE};
        rotateLeft = false;
        rotateRight = false;
    }
    
    public Spindexer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        rightColorSensor = new ArtifactDetector(hardwareMap, "sensor_color");
        leftColorSensor = new ArtifactDetector(hardwareMap, "leftColorSensor");
        
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class,
                                              "rightDistanceSensor");
        
        
    }
    
    // Rotates spindexer 3 times to drop all artifacts
    public void spinToDropAllArtifacts() {
        rotateToSlot(dropAllIndex);
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
    
    // +1 is to the right, clockwise
    public boolean intakeArtifact(Artifact artifact) {
        int count = getNumberOfArtifacts();
        slots[count] = artifact;
        
        switch (count) {
            case 0:
                rotateToSlot(1);
                break;
            case 1:
                rotateToSlot(2);
                break;
            case 2:
                if (slots[1] == artifact) {
                    rotateToSlot(2.5);
                    dropAllIndex = 0;
                }
                else {
                    rotateToSlot(1.5);
                    dropAllIndex = -1;
                }
                return true;
        }
        return false;
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
    
    private Integer chooseRotationTarget(Artifact artifact) {
        int leftIndex = getLeftIndex(currentSlotIndex);
        int rightIndex = getRightIndex(currentSlotIndex);
        
        int leftSlotIndex = rollIndex(leftIndex);
        int rightSlotIndex = rollIndex(rightIndex);
        
        boolean leftIsArtifact = slots[leftSlotIndex] == artifact;
        boolean rightIsArtifact = slots[rightSlotIndex] == artifact;
        
        if (!(leftIsArtifact || rightIsArtifact)) {
            rotateLeft = rotateRight = false;
            return null;
        }
        
        if (leftIsArtifact && rightIsArtifact && (rotateLeft || rotateRight)) {
            // keep previous direction
        } else if (leftIsArtifact) {
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
        slots[slotIndex] = Artifact.NONE;
        return true;
    }
    
    public void rotateToArtifact(Artifact artifact) {
        Integer target = chooseRotationTarget(artifact);
        if (target == null) return;
        
        rotateToSlot(target);
    }
    
    public void forceRotateToArtifact(Artifact artifact) {
        if (!_rotateToArtifact(artifact)) { // rotates to opposite color if not found
            _rotateToArtifact(artifact.oppositeColor());
        }
    }
    
    public void rotateToSlot(double slotIndex) {
        
        // .472
//        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .483);
        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .472);
        
        currentSlotIndex = slotIndex;
        
        
    }
    
    public int getNumberOfArtifacts() {
        return (slots[0].isArtifact() ? 1 : 0)
            + (slots[1].isArtifact() ? 1 : 0)
            + (slots[2].isArtifact() ? 1 : 0);
    }
    
    public static int rollIndex(int index) {
        return Math.floorMod(index, 3);
    }
    
}
