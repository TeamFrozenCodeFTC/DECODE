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

    public Artifact[] artifacts = Artifact.getEmptyPattern();

    public double currentSlotIndex = 0;

    public void reset() {
        artifacts = Artifact.getEmptyPattern();
        lastDirection = null;
        rotateToSlot(0);
        lastArtifactPresent = true;
        waitingForDrop = false;
    }
    
    public int shiftLeft(int steps) {
        int result = (int)Math.ceil(currentSlotIndex);
        return result - steps;
    }
    
    public int shiftRight(int steps) {
        int result = (int)Math.floor(currentSlotIndex);
        return result + steps;
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
    
    public Spindexer(HardwareMap hardwareMap) {
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
        int leftIndex = shiftLeft(1);
        int rightIndex = shiftRight(1);

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
    
    public void rotateToSlot(double slotIndex) {
//        if (slotIndex == currentSlotIndex) return; BROKEN
//
        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .483);

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
    
    public boolean lastArtifactPresent = true;
    public boolean waitingForDrop = false;
    
    public void rotateAndArmDrop(double slotIndex) {
        if (waitingForDrop) return;
        
        rotateToSlot(slotIndex);
        waitingForDrop = true;
    }
    
    public boolean didArtifactJustDrop() {
        boolean artifactPresent =
            leftDistanceSensor.getDistance(DistanceUnit.INCH) <= 6.5 ||
                rightDistanceSensor.getDistance(DistanceUnit.INCH) <= 6.5;
        
        boolean dropped = waitingForDrop && lastArtifactPresent && !artifactPresent;
        
        lastArtifactPresent = artifactPresent;
        
        if (dropped) {
            waitingForDrop = false;
        }
        
        return dropped;
    }
    
    
    //    public void rotateToSlot(double slotIndex) {
//        if (slotIndex == currentSlotIndex) return;
//
//        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .543);
//        currentSlotIndex = slotIndex;
//        state = SpindexerNew.State.ROTATING;
//    }
//
//    public void update() {
//        if (state == SpindexerNew.State.ROTATING &&
//            !partialRotationMagnetSensor.isPressed() &&
//            !fullRotationMagnetSensor.isPressed()) {
//
//            spindexerHasMoved = true;
//            rotationTimer.reset();
//        }
//
//        if (!spindexerHasMoved || rotationTimer.seconds() < 0.05) return;
//
//        if (isPartialRotation()
//            ? partialRotationMagnetSensor.isPressed()
//            : fullRotationMagnetSensor.isPressed()) {
//
//            state = isPartialRotation()
//                ? SpindexerNew.State.PARTIAL_ROTATION
//                : SpindexerNew.State.FULLY_ROTATED;
//        }
//
//        spindexerHasMoved = false;
//    }
//
//    public boolean isRotating() {
//        return state == SpindexerNew.State.ROTATING;
//    }
}
