package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.utils.Timeout;

public class Spindexer {
    public ServoImplEx servo;
    public ArtifactDetector artifactDetector;
    public DistanceSensors distanceSensors;
    
    public Artifact[] artifacts = Artifact.getEmptyPattern();
    
    public double currentSlotIndex = 0;

    public Direction lastDirection = null;

    public Spindexer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        
        artifactDetector = new ArtifactDetector(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
    }
    
    public Artifact getCurrentArtifact() {
        int slotIndex = rollIndex((int) currentSlotIndex);
        return artifacts[slotIndex];
    }
    
    public Artifact getDetectedArtifact() {
        artifactDetector.update();
        return artifactDetector.getDetectedArtifact();
    }
    
    public boolean isSpindexerClear() {
        distanceSensors.update();
        return distanceSensors.isSpindexerClear();
    }
    
    public boolean isArtifactInHandoffZone() {
        distanceSensors.update();
        return distanceSensors.isArtifactInHandoffZone();
    }
    
    public boolean lastArtifactPresent = true;
    public boolean waitingForDrop = false;
    public Timeout firingTimer = new Timeout();
    
//    public void rotateAndDrop(double slotIndex) {
//        if (waitingForDrop) return;
//
//        rotateToSlot(slotIndex);
//        firingTimer.resetAndStart();
//        //lastArtifactPresent = true;
//        lastArtifactPresent = !isSpindexerClear(); // NEW capture truth ONCE
//        waitingForDrop = true;
//    }
    
    boolean hadArtifactInitially;
    
    public void rotateAndDrop(double slotIndex) {
        if (waitingForDrop) return;
        
        rotateToSlot(slotIndex);
        
        boolean clear = isSpindexerClear();
        lastArtifactPresent = !clear;
        hadArtifactInitially = lastArtifactPresent;
        
        clearCycles = 0;
        firingTimer.resetAndStart();
        waitingForDrop = true;
    }
    
    private int clearCycles = 0;
    private static final int REQUIRED_CLEAR_CYCLES = 3;
    
    public boolean didArtifactJustDrop() {
        if (!waitingForDrop) return false;
        if (firingTimer.seconds() < 0.05) return false;
        
        boolean clear = isSpindexerClear();
        
        if (clear) {
            clearCycles++;
        } else {
            clearCycles = 0;
        }
        
        boolean dropped =
            hadArtifactInitially && lastArtifactPresent && clearCycles >= REQUIRED_CLEAR_CYCLES;
        
        if (dropped) {
            waitingForDrop = false;
            clearCycles = 0;
        }
        
        lastArtifactPresent = !clear;
        return dropped;
    }
    
    public boolean didSlotStartEmpty() {
        return waitingForDrop && !hadArtifactInitially;
    }
    
    //    public boolean didArtifactJustDrop() {
//        boolean isSpindexerClear = isSpindexerClear();
//
//        boolean dropped =
//            firingTimer.seconds() > 0.1 && waitingForDrop && lastArtifactPresent && isSpindexerClear;
//
//        lastArtifactPresent = !isSpindexerClear;
//
//        if (dropped) {
//            waitingForDrop = false;
//        }
//
//        return dropped;
//    }
    
    public void reset() {
        artifacts = Artifact.getEmptyPattern();
        lastDirection = null;
        waitingForDrop = false;
        rotateToSlot(0);
    }
    
    public int shiftLeft(int steps) {
        int result = (int)Math.ceil(currentSlotIndex);
        return result - steps;
    }
    
    public int shiftRight(int steps) {
        int result = (int)Math.floor(currentSlotIndex);
        return result + steps;
    }
    
    public void intakeArtifact(Artifact artifact, MotifPattern motif) {
        int count = getNumberOfArtifacts();
        
        switch (count) {
            case 1:
            case 2:
                rotateLeft();
                break;
            case 3:
                if ((motif != MotifPattern.PGP && getLeftArtifact() == artifact)
                    || (motif == MotifPattern.PGP && getRightArtifact() != artifact)) {
                    rotatePartialLeft();
                } else {
                    rotatePartialRight();
                }
                break;
            }
    }
    
    public enum Direction {
        LEFT,
        RIGHT
    }
    
    public Artifact getLeftArtifact() {
        int leftIndex = shiftLeft(1);
        int leftSlotIndex = rollIndex(leftIndex);
        return artifacts[leftSlotIndex];
    }
    
    public Artifact getRightArtifact() {
        int rightIndex = shiftRight(1);
        int rightSlotIndex = rollIndex(rightIndex);
        return artifacts[rightSlotIndex];
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
    
    public void rotateToSlot(double slotIndex) {
        //        if (slotIndex == currentSlotIndex) return; BROKEN
        //
        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .495);
        
        currentSlotIndex = slotIndex;
    }
    
    public void rotateLeft() {
        rotateToSlot(currentSlotIndex + 1);
    }
    
    public void rotatePartialLeft() {
        rotateToSlot(currentSlotIndex + .5);
    }
    
    public void rotateRight() {
        rotateToSlot(currentSlotIndex - 1);
    }
    
    public void rotatePartialRight() {
        rotateToSlot(currentSlotIndex - .5);
    }
    
    public void disable() {
        servo.setPwmDisable();
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
