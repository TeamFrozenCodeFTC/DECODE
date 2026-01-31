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
    
//    private boolean prevSpindexerClear = true;
//    private boolean prevArtifactInHandoff = false;
//
    public Direction lastDirection = null;
//
//    private final Poller<Boolean> artifactDroppedPoller =
//        new Poller<>(() -> {
//            distanceSensors.update();
//            boolean artifactDroppedEvent = !prevSpindexerClear && isSpindexerClear();
//            prevSpindexerClear = isSpindexerClear();
//            return artifactDroppedEvent;
//        }, 100, 0);
//
//    private final Poller<Boolean> artifactEnteredSpindexerPoller =
//        new Poller<>(() -> {
//            distanceSensors.update();
////            boolean artifactEnteredEvent =
////                !prevArtifactInHandoff && isArtifactInHandoffZone();
////            prevArtifactInHandoff = isArtifactInHandoffZone();
//            boolean artifactEnteredEvent =
//                !prevArtifactInHandoff && isArtifactInHandoffZone();
//            prevArtifactInHandoff = isSpindexerClear();
//            return artifactEnteredEvent;
//        }, 200, 0);
//
    public Spindexer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        
        artifactDetector = new ArtifactDetector(hardwareMap);
        distanceSensors = new DistanceSensors(hardwareMap);
    }
    
    public Artifact getCurrentArtifact() {
        int slotIndex = rollIndex((int) currentSlotIndex);
        return artifacts[slotIndex];
    }
    
//    public boolean didArtifactJustDrop() {
//        return artifactDroppedPoller.poll();
//    }
//
//    public boolean didArtifactJustEnterSpindexer() {
//        return artifactEnteredSpindexerPoller.poll();
//    }
    
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
    
    public void rotateAndDrop(double slotIndex) {
        if (waitingForDrop) return;
        
        rotateToSlot(slotIndex);
        firingTimer.resetAndStart();
        //lastArtifactPresent = true;
        waitingForDrop = true;
    }
    
    public boolean didArtifactJustDrop() {
        boolean isSpindexerClear = isSpindexerClear();
        
        boolean dropped =
            firingTimer.seconds() > 0.1 && waitingForDrop && lastArtifactPresent && isSpindexerClear;
        
        lastArtifactPresent = !isSpindexerClear;
        
        if (dropped) {
            waitingForDrop = false;
        }
        
        return dropped;
    }
    
    public void reset() {
        artifacts = Artifact.getEmptyPattern();
        lastDirection = null;
        rotateToSlot(0);
//        prevSpindexerClear = false;
//        prevArtifactInHandoff = false;
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
        servo.setPosition(slotIndex * ((double) 120 / (360*4.5)) + .483);
        
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
