package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Artifact;

import java.util.OptionalInt;

//enum State {
//    IDLE,
//    ROTATING,
//    HOLDING
//}

public class Spindexer2 {
    // +1 is to the right, clockwise
    
    public enum State {
        PARTIAL_ROTATION,
        FULLY_ROTATED,
        ROTATING
    }
    
    public enum Direction {
        LEFT,
        RIGHT
    }
    
    public Direction lastDirection = null;
    
    private State state = null;
    
    public ServoImplEx servo;
    public TouchSensor fullRotationMagnetSensor;
    public TouchSensor partialRotationMagnetSensor;

    public Artifact[] artifacts = Artifact.getEmptyPattern();
    
    public double currentSlotIndex = 0;
    
    boolean spindexerHasMoved = false;
    ElapsedTime rotationTimer = new ElapsedTime();
    
    public void resetSlots() {
        artifacts = Artifact.getEmptyPattern();
        lastDirection = null;
    }
    
    public int shiftLeft(int steps) {
        int result = (int)Math.ceil(currentSlotIndex);
        return result - steps;
    }
    
    public int shiftRight(int steps) {
        int result = (int)Math.floor(currentSlotIndex);
        return result + steps;
    }
    
    public Spindexer2(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        fullRotationMagnetSensor = hardwareMap.get(TouchSensor.class, "fullRotationMagnetSensor");
        partialRotationMagnetSensor = hardwareMap.get(TouchSensor.class,
                                                      "partialRotationMagnetSensor");
    }

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
    
    public OptionalInt findBestRotationToArtifact2(Artifact artifact) {
        int leftIndex = shiftLeft(1);
        int rightIndex = shiftRight(1);
        
        int leftSlotIndex = rollIndex(leftIndex);
        int rightSlotIndex = rollIndex(rightIndex);
        
        boolean leftIsArtifact = artifacts[leftSlotIndex] == artifact;
        boolean rightIsArtifact = artifacts[rightSlotIndex] == artifact;
        
        if (!(leftIsArtifact || rightIsArtifact)) {
            return OptionalInt.empty();
        }
        
        if (leftIsArtifact && rightIsArtifact && lastDirection != null) {
            // keep direction
        } else if (leftIsArtifact) {
            lastDirection = Direction.LEFT;
        } else {
            lastDirection = Direction.RIGHT;
        }
        
        return OptionalInt.of(lastDirection == Direction.LEFT ? leftIndex : rightIndex);
    }
    
    public void rotateToSlot(double slotIndex) {
        if (slotIndex == currentSlotIndex) return;
        
        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .505);
        currentSlotIndex = slotIndex;
        state = State.ROTATING;
    }
    
    public void update() {
        if (state == State.ROTATING &&
            !partialRotationMagnetSensor.isPressed() &&
            !fullRotationMagnetSensor.isPressed()) {
            
            spindexerHasMoved = true;
            rotationTimer.reset();
        }
        
        if (!spindexerHasMoved || rotationTimer.seconds() < 0.05) return;
        
        if (isPartialRotation()
            ? partialRotationMagnetSensor.isPressed()
            : fullRotationMagnetSensor.isPressed()) {
            
            state = isPartialRotation()
                ? State.PARTIAL_ROTATION
                : State.FULLY_ROTATED;
        }
        
        spindexerHasMoved = false;
    }

    public boolean isRotating() {
        return state == State.ROTATING;
    }
    
    public boolean isPartialRotation() {
        return currentSlotIndex % 1 != 0;
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
