package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Artifact;

public class XXSpindexer {
    // public static double DDEG120 = 0.0667;
    public static double THIRD_SERVO = 0.074;
    public static double SIXTH_SERVO = THIRD_SERVO / 2.0;

    public enum RotationAmount {
        THIRD,
        SIXTH
    }

    public ServoImplEx servo;
    public ArtifactDetector rightColorSensor;
    public ArtifactDetector leftColorSensor;
    double offset = 0.0;

    public Artifact[] slots = Artifact.getEmptyPattern();
    public double currentSlotIndex = 2;
    public double currentRotationDegrees = 0.0;
    public double currentServoPosition = 0.0;
    public OpMode op;

    public XXSpindexer(OpMode mode) {
        this(mode, 0.0);
    }

    public XXSpindexer(OpMode mode, double offset) {
        this.op = mode;
        servo = op.hardwareMap.get(ServoImplEx.class, "spindexer");
        rightColorSensor = new ArtifactDetector(op.hardwareMap, "sensor_color");
        leftColorSensor = new ArtifactDetector(op.hardwareMap, "leftColorSensor");
        setOffset(offset);
    }

    public void resetSlots() {
        slots = new Artifact[]
                {Artifact.NONE, Artifact.NONE, Artifact.NONE};
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void adjustServo(double amount) {
        servo.setPosition(this.currentServoPosition + amount);
    }

    public void lockAdjustment(double amount) {
        this.offset += amount;
        this.currentServoPosition += amount;
        servo.setPosition(this.currentServoPosition);
    }

    public void reset() {
        // can rotate 1800 degrees. Rotate to halfway + offset
        currentServoPosition = 0.5 + offset;
        servo.setPosition(currentServoPosition);
        currentRotationDegrees = 1800.0 / 2.0;
        currentSlotIndex = 2;
    }

    public Artifact getDetectedArtifact() {
        Artifact detected = rightColorSensor.getDetectedArtifact();
        if (detected == Artifact.NONE) {
            return leftColorSensor.getDetectedArtifact();
        }
        return detected;
    }

    public boolean canRotateRight(RotationAmount rotation) {
        double amount = rotation == RotationAmount.THIRD ? THIRD_SERVO : SIXTH_SERVO;
        return currentServoPosition + amount < 0.9;
    }

    public boolean canRotateLeft(RotationAmount rotation) {
        double amount = rotation == RotationAmount.THIRD ? THIRD_SERVO : SIXTH_SERVO;
        return currentServoPosition - amount > 0.1;
    }

    public long rotateLeft(RotationAmount rotation) {
        long delay = 0;

        double slotChange = 1.0;
        if(rotation == RotationAmount.SIXTH) {
            slotChange = 0.5;
        }

        double targetSlot = currentSlotIndex - slotChange;
        if (targetSlot < 0) {
            targetSlot = 3 - slotChange;
        }

        if(!canRotateLeft(rotation)) {
            reset();
            delay += 2500;
        }

        op.telemetry.addData("current", currentSlotIndex);
        op.telemetry.addData("target", targetSlot);
        op.telemetry.update();

        delay += doRotation(targetSlot);

        return delay;
    }

    public long rotateRight(RotationAmount rotation) {
        long delay = 0;

        double slotChange = 1.0;
        if(rotation == RotationAmount.SIXTH) {
            slotChange = 0.5;
        }

        double targetSlot = currentSlotIndex + slotChange;
        if (targetSlot > 2.5) {
            targetSlot = targetSlot - 3;
        }

        if(!canRotateRight(rotation)) {
            reset();
            delay += 2500;
        }
        delay += doRotation(targetSlot);

        return delay;
    }

    private long doRotation(double targetSlot) {
        if(currentSlotIndex == targetSlot) {
            return 0;
        }

        double amount = 0.0;
        if(currentSlotIndex == 0 && targetSlot == 2.5) amount = -1 * SIXTH_SERVO;
        else if(currentSlotIndex == 0 && targetSlot == 2) amount = -1 * THIRD_SERVO;
        else if(currentSlotIndex == 0.5 && targetSlot == 0) amount = -1 * SIXTH_SERVO;
        else if(currentSlotIndex == 0.5 && targetSlot == 2.5) amount = -1 * THIRD_SERVO;
        else if(currentSlotIndex == 1 && targetSlot == 0.5) amount = -1 * SIXTH_SERVO;
        else if(currentSlotIndex == 1 && targetSlot == 0) amount = -1 * THIRD_SERVO;
        else if(currentSlotIndex == 1.5 && targetSlot == 1) amount = -1 * SIXTH_SERVO;
        else if(currentSlotIndex == 1.5 && targetSlot == 0.5) amount = -1 * THIRD_SERVO;
        else if(currentSlotIndex == 2 && targetSlot == 1.5) amount = -1 * SIXTH_SERVO;
        else if(currentSlotIndex == 2 && targetSlot == 1) amount = -1 * THIRD_SERVO;
        else if(currentSlotIndex == 2.5 && targetSlot == 2) amount = -1 * SIXTH_SERVO;
        else if(currentSlotIndex == 2.5 && targetSlot == 1.5) amount = -1 * THIRD_SERVO;
        else if(currentSlotIndex == 0 && targetSlot == 0.5) amount = SIXTH_SERVO;
        else if(currentSlotIndex == 0 && targetSlot == 1) amount = THIRD_SERVO;
        else if(currentSlotIndex == 0.5 && targetSlot == 1) amount = SIXTH_SERVO;
        else if(currentSlotIndex == 0.5 && targetSlot == 1.5) amount = THIRD_SERVO;
        else if(currentSlotIndex == 1 && targetSlot == 1.5) amount = SIXTH_SERVO;
        else if(currentSlotIndex == 1 && targetSlot == 2) amount = THIRD_SERVO;
        else if(currentSlotIndex == 1.5 && targetSlot == 2) amount = SIXTH_SERVO;
        else if(currentSlotIndex == 1.5 && targetSlot == 2.5) amount = THIRD_SERVO;
        else if(currentSlotIndex == 2 && targetSlot == 2.5) amount = SIXTH_SERVO;
        else if(currentSlotIndex == 2 && targetSlot == 0) amount = THIRD_SERVO;
        else if(currentSlotIndex == 2.5 && targetSlot == 0) amount = SIXTH_SERVO;
        else if(currentSlotIndex == 2.5 && targetSlot == 0.5) amount = THIRD_SERVO;

        currentServoPosition = currentServoPosition + amount;
        servo.setPosition(currentServoPosition);
        currentSlotIndex = targetSlot;

        return 500;
    }

    public void pushArtifact(Artifact artifact) {
        double index = Math.floor(currentSlotIndex);
        if(index == currentSlotIndex) {
            slots[(int) index] = artifact;
        }
    }

    public void popArtifact() {
        double index = Math.floor(currentSlotIndex);
        if(index == currentSlotIndex) {
            slots[(int) index] = Artifact.NONE;
        }
    }

    public boolean currentSlotEmpty() {
        boolean empty = false;
        double index = Math.floor(currentSlotIndex);
        if(index == currentSlotIndex) {
            empty = slots[(int) index] == Artifact.NONE;
        }
        return empty;
    }

    public boolean isFullyRotated() {
        return currentSlotIndex == Math.floor(currentSlotIndex);
    }

    public boolean allSlotsOccupied() {
        boolean full = true;
        for(Artifact a : slots) {
            if(a == Artifact.NONE) {
                full = false;
                break;
            }
        }
        return full;
    }

    public String getLoadedPattern() {
        String pattern = "";

        for(Artifact a : slots) {
            switch(a) {
                case GREEN:
                    pattern += "G";
                    break;
                case PURPLE:
                    pattern += "P";
                    break;
                default:
                    pattern += "N";
            }
        }

        return pattern;
    }
}
