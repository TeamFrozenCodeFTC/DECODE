package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;


// boolean hasBallDetected();
//boolean isStationary();

public class Indexer {
    
    public ServoImplEx servo;
    public ArtifactDetector rightColorSensor;
    public ArtifactDetector leftColorSensor;
    public Paddles paddles;

    public DistanceSensor rightDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public TouchSensor fullRotationMagnetSensor;
    public TouchSensor partialRotationMagnetSensor;

    public boolean spindexerIsRotating = false;
    public boolean spindexerHasMoved = false;

    public Artifact[] artifacts = Artifact.getEmptyPattern();

    public double currentSlotIndex = 0;

    public void resetSlots() {
        artifacts = Artifact.getEmptyPattern();
        rotateLeft = false;
        rotateRight = false;
    }

    public int shiftLeft(int steps) {
        int result = (int) Math.ceil(currentSlotIndex);
        return result - steps;
    }

    public int shiftRight(int steps) {
        int result = (int) Math.floor(currentSlotIndex);
        return result + steps;
    }

    public Indexer(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "spindexer");
        rightColorSensor = new ArtifactDetector(hardwareMap, "sensor_color");
        leftColorSensor = new ArtifactDetector(hardwareMap, "leftColorSensor");

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class,
                                              "rightDistanceSensor");
        fullRotationMagnetSensor = hardwareMap.get(TouchSensor.class, "fullRotationMagnetSensor");
        partialRotationMagnetSensor = hardwareMap.get(TouchSensor.class,
                                                      "partialRotationMagnetSensor");
        paddles = new Paddles(hardwareMap);
    }

    public Artifact getDetectedArtifact() {
        Artifact detected = rightColorSensor.getDetectedArtifact();
        if (detected == Artifact.NONE) {
            return leftColorSensor.getDetectedArtifact();
        }
        return detected;
    }

    public boolean artifactIsInSpindexer() {
        int distanceThreshold = 3;
        return leftDistanceSensor.getDistance(DistanceUnit.INCH) < distanceThreshold
            || rightDistanceSensor.getDistance(DistanceUnit.INCH) < distanceThreshold;
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
                } else {
                    rotateToSlot(1.5);
                }
                return true;
        }
        return false;
    }

    public boolean rotateLeft = false;
    public boolean rotateRight = false;

    public Integer chooseRotationTarget(Artifact artifact) {
        int leftIndex = shiftLeft(1);
        int rightIndex = shiftRight(1);

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

    public boolean rotateToArtifact(Artifact artifact) {
        Integer target = chooseRotationTarget(artifact);
        if (target == null) return false;

        rotateToSlot(target);
        return true;
    }

    public void rotateToSlot(double slotIndex) {
        if (slotIndex == currentSlotIndex) return;
        spindexerIsRotating = true;
        servo.setPosition(slotIndex * ((double) 120 / (360 * 4.5)) + .485);
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

    Artifact incomingArtifact = Artifact.NONE;

    public void update() {
        if (spindexerIsRotating && !fullRotationMagnetSensor.isPressed()) {
            spindexerHasMoved = true;
        }
        if (spindexerHasMoved && fullRotationMagnetSensor.isPressed()) {
            spindexerIsRotating = false;
            spindexerHasMoved = false;
        }

        if (incomingArtifact.isArtifact() && !spindexerIsRotating) {
            ///
        }


        Artifact detectedArtifact = getDetectedArtifact();
        boolean artifactIsInSpindexer = artifactIsInSpindexer();

        if (detectedArtifact.isArtifact()) {
            if (!incomingArtifact.isArtifact()) {
                incomingArtifact = detectedArtifact;

                if (getNumberOfArtifacts() == 2 && !artifactIsInSpindexer) {
                    paddles.close();
                }
            }
        }

        if (incomingArtifact.isArtifact() && artifactIsInSpindexer && !spindexerIsRotating
                    && stateTimer.seconds() > 0.25) {

        }
            spindexer.intakeArtifact(intakedArtifact);
        intakedArtifact = Artifact.NONE;

                    if (spindexer.getNumberOfArtifacts() == 3) {
        setState(Robot.State.REVVING);
                    }
                        }
                        else if (spindexerIsRotating && !artifactIsInSpindexer) {
            stateTimer.resetAndStart();
        spindexerIsRotating = false;
    }
}
//flywheel.stop();
//        intakeRamp.uptake();
//        intake.intake();
//        stateTimer.resume();
//
//Artifact detectedArtifact2 = spindexer.getDetectedArtifact();
//
//boolean artifactIsInSpindexer = spindexer.artifactIsInSpindexer();
//
//        if (detectedArtifact2.isArtifact()) {
//    if (!intakedArtifact.isArtifact()) {
//intakedArtifact = detectedArtifact2;
////spindexer.intakeArtifact(intakedArtifact);
//            }
//
//                if (spindexer.getNumberOfArtifacts() == 2 && !artifactIsInSpindexer) {
//    paddles.close();
//            }
//                }
//
//                if (intakedArtifact.isArtifact() && !spindexerIsRotating && artifactIsInSpindexer
//            && stateTimer.seconds() > 0.25) {
//spindexerIsRotating = true;
//    spindexer.intakeArtifact(intakedArtifact);
//intakedArtifact = Artifact.NONE;
//
//            if (spindexer.getNumberOfArtifacts() == 3) {
//setState(Robot.State.REVVING);
//            }
//                }
//                else if (spindexerIsRotating && !artifactIsInSpindexer) {
//    stateTimer.resetAndStart();
//spindexerIsRotating = false;
//    }
