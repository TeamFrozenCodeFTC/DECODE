package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Indexer {
    public DcMotorEx motor;
    
    public enum Slot {
        PURPLE,
        GREEN,
        EMPTY,
        UNKNOWN
    }
    
    public Slot[] slots = new Slot[]{Slot.EMPTY, Slot.EMPTY, Slot.EMPTY};

    public int currentSlot = 1;
    
    public Indexer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "indexer");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public boolean artifactIsDetected() {
        return false;
    }
    
    public boolean isAtMaxCapacity() {
        return slots[0] != Slot.EMPTY && slots[1] != Slot.EMPTY && slots[2] != Slot.EMPTY;
    }
    
    public int getNumberOfArtifacts() {
        return (slots[0] != Slot.EMPTY ? 1 : 0)
            + (slots[1] != Slot.EMPTY ? 1 : 0)
            + (slots[2] != Slot.EMPTY ? 1 : 0);
    }
    
    
    public void rotateToPurple() {
    
    }
    
    public void rotateToGreen() {
    
    }
    
    public void rotateToArtifact() {
    
    }
    
    public void rotatePartiallyToArtifact() {
    
    }
    
    public void dropCurrentSlot() {
        motor.setTargetPosition(0);
    }
    
    public void rotateToNextEmptySlot() {
        motor.setTargetPosition(motor.getTargetPosition() + 360/3);
    }
    
    public void lockArtifacts() {
        motor.setTargetPosition(motor.getTargetPosition() + 360/3/2);
    }
    
    public Slot getCurrentSlot() {
        return slots[currentSlot - 1];
    }
}
