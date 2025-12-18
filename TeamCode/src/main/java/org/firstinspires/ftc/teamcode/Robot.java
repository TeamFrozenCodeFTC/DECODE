package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Paddles;

import java.util.Arrays;
import java.util.Collections;

public class Robot {
    public Intake intake;
    public Flywheel flywheel;
    public Spindexer spindexer;
    public Follower follower;
    public Ramp intakeRamp;
    public Paddles paddles;
    
    public AllianceColor allianceColor = AllianceColor.BLUE;
    public Artifact[] motifPattern = new Artifact[]
        {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};

    public boolean spindexerIsRotating = false;
    public boolean paddlesRotatedUp = false;

    public int firedArtifacts = 0;
    public int artifactsToFire = 0;
    
    public boolean paddlesRotatingDown = false;
    public boolean reverseSpindexerCase = false;
    public boolean droppedFirstArtifact = false;

    public boolean isAuto = false;
    
    public Artifact intakedArtifact = Artifact.NONE;
    
    public Robot.State state = Robot.State.IDLE;
    Timeout stateTimer = new Timeout();

    public enum State {
        GROUND_FIRE,
        LOAD_ARTIFACTS,
        SENSOR_LOAD_ARTIFACTS,
        FIRING,
        FAST_FIRING,
        REVVING,
        IDLE,
        SALVO,
    }
    
    public void preload(Artifact[] artifacts) {
        spindexer.slots = artifacts;
        artifactsToFire = spindexer.getNumberOfArtifacts();
        firedArtifacts = 0;
        droppedFirstArtifact = false;
        updateReverseCase();
    }
    
    public void setState(Robot.State newState) {
        if (state == newState) {
            return;
        }
        state = newState;
        stateTimer.pauseAtZero();
    }

    public Robot(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        intakeRamp = new Ramp(hardwareMap);
        paddles = new Paddles(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        
        flywheel.filteredVoltage = follower.getVoltage();
    }

    private void revLauncher() {
        flywheel.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition()));
    }
    
    public void revTowardGoal() {
        revLauncher();
        follower.lockHeadingAt(getAngleToGoal());
    }
    
    public void updateReverseCase() {
        boolean isPGPMotif =
            Arrays.equals(motifPattern, new Artifact[]{ Artifact.PURPLE, Artifact.GREEN,
                Artifact.PURPLE });
        
        boolean hasPGP =
            Collections.frequency(Arrays.asList(spindexer.slots), Artifact.GREEN) == 1
                && Collections.frequency(Arrays.asList(spindexer.slots), Artifact.PURPLE) == 2;
        
        reverseSpindexerCase = isPGPMotif && hasPGP;
    }
    
    public void fireThrough() {
        stateTimer.resume();
        intakeRamp.intakeThrough();
        paddles.open();
        
        revTowardGoal();
        
        if (flywheel.isUpToSpeed()) {
            intake.intake();
        }
    }
    
    public void sensorLoadArtifacts() {
        flywheel.stop();
        intakeRamp.uptake();
        intake.intake();
        
        Artifact detectedArtifact = spindexer.getDetectedArtifact();
        
        if (detectedArtifact.isArtifact()) {
            intakedArtifact = detectedArtifact;
            paddles.close();
            intake.stop();
        }
        
        if (spindexer.artifactIsInSpindexer() && !spindexerIsRotating && intakedArtifact.isArtifact()) {
            spindexerIsRotating = true;
            
            artifactsToFire++;
            
            boolean isFull = spindexer.intakeArtifact(intakedArtifact);
            
            if (isFull) {
                intake.stop();
                intakeRamp.outtake();
                spindexerIsRotating = false;
                intakedArtifact = Artifact.NONE;
                updateReverseCase();
                setState(State.REVVING);
            }
        }
        else if (!spindexer.artifactIsInSpindexer() && spindexerIsRotating) {
            spindexerIsRotating = false;
            intakedArtifact = Artifact.NONE;
            paddles.open();
        }
    }
    
    public void motifFire() {
        if (flywheel.artifactLaunched()) {
            int droppedIndex = Spindexer.rollIndex((int) spindexer.currentSlotIndex);
            spindexer.slots[droppedIndex] =
                Artifact.NONE;
            firedArtifacts++;
        }
        
        if (flywheel.isUpToSpeed()) {
            if (spindexer.getNumberOfArtifacts() == 0) {
                spindexer.resetSlots();
                spindexer.rotateToSlot(0);
                firedArtifacts = 0;
                setState(State.IDLE);
                return;
            }
            
            spindexer.rotateToArtifact(motifPattern[firedArtifacts]);
        }
        
        stateTimer.resume();
        
        revTowardGoal();
        intakeRamp.outtake();
        intake.stop();
        
//        if (stateTimer.seconds() > 0.7) { // was .5
//            paddles.open();
//        }
    }
 
    public void update() {
        follower.update();
        
        switch (state) {
            case IDLE:
                droppedFirstArtifact = false;
                
                stateTimer.resume();
                
                if (stateTimer.seconds() > 0.5) {
                    paddles.open();
                }
                
                flywheel.stop();
                intake.stop();
                follower.lockHeadingAt(null);
                
                if (spindexer.getNumberOfArtifacts() == 0) {
                    spindexer.rotateToSlot(0);
                }
                break;
            case GROUND_FIRE:
                fireThrough();
                break;
            case LOAD_ARTIFACTS:
                originalLoadArtifacts();
                break;
            case SENSOR_LOAD_ARTIFACTS: // make this not triangle or make robot always
                // intake
                sensorLoadArtifacts();
                break;
            case FIRING: // Slowly launches artifacts according to motif pattern
                motifFire();
                break;
            case SALVO: // Launches artifacts as fast as possible
                if (flywheel.artifactLaunched()) {
                    spindexer.slots[spindexer.getNumberOfArtifacts() - 1] = Artifact.NONE;
                }
                
                if (flywheel.isUpToSpeed()) {
                    spindexer.rotateToSlot(spindexer.dropAllIndex);
                    
                    if (spindexer.getNumberOfArtifacts() == 0) {
                        setState(State.IDLE);
                        break;
                    }
                }
            case REVVING:
                revTowardGoal();
                intakeRamp.outtake();
                intake.stop();
                
                stateTimer.resume();

                if (stateTimer.seconds() > 0.7) { // was .5
                    paddles.open();
                }
                break;
        }
        
        intake.update(follower.getMotionState().deltaTime);

        flywheel.update(follower.getMotionState().deltaTime, follower.getVoltage());
    }
    
    public void originalMotifFire() {
        revTowardGoal();
        intakeRamp.outtake();
        intake.stop();
        
        if (stateTimer.seconds() > 0.5) {
            paddles.open();
        }
        
        if (flywheel.isUpToSpeed()) {
            stateTimer.resume();
            
            if (!droppedFirstArtifact) {
                spindexer.forceRotateToArtifact(motifPattern[0]);
                droppedFirstArtifact = true;
            }
            
            if (!reverseSpindexerCase || firedArtifacts >= 1) {
                spindexer.forceRotateToArtifact(motifPattern[1]);
                spindexer.forceRotateToArtifact(motifPattern[2]);
            }
        }
        
        if (droppedFirstArtifact && (flywheel.artifactLaunched())) {
            firedArtifacts++;
            artifactsToFire--;
            if (artifactsToFire <= 0) {
                resetSpindexer();
                return;
            }
        }
        
        if (isAuto && stateTimer.seconds() > 2.5) {
            resetSpindexer();
        }
        if (!isAuto && stateTimer.seconds() > 6) {
            resetSpindexer();
        }
    }
    
    
    public void originalLoadArtifacts() {
        flywheel.stop();
        intakeRamp.uptake();
        intake.intake();
        
        if (spindexerIsRotating) {
            paddles.open(); // down
        }
        else {
            intake.stop();
            paddles.close();
        }
        
        Artifact detectedArtifact = spindexer.getDetectedArtifact();
        
        if (detectedArtifact.isArtifact()) {
            if (paddlesRotatingDown) {
                intake.outtake();
            }
            
            if (stateTimer.isPaused()) {
                intakedArtifact = detectedArtifact;
                spindexerIsRotating = false;
                stateTimer.resetAndStart();
                paddles.close();
            }
        }
        else {
            paddlesRotatingDown = false;
        }
        
        if (stateTimer.seconds() > 1.1) { // was 1.3
            spindexerIsRotating = true;
            paddlesRotatedUp = false;
            stateTimer.pauseAtZero();
            intakedArtifact = Artifact.NONE;
            paddlesRotatingDown = true;
        } else if ((stateTimer.seconds() > 0.4) && !paddlesRotatedUp) {
            paddlesRotatedUp = true;
            
            artifactsToFire++;
            
            boolean isFull = spindexer.intakeArtifact(intakedArtifact);
            
            if (isFull) {
                intake.stop();
                intakeRamp.outtake();
                spindexerIsRotating = true;
                paddlesRotatedUp = false;
                intakedArtifact = Artifact.NONE;
                updateReverseCase();
                setState(State.REVVING);
            }
        }
    }
    
    public void resetSpindexer() {
        artifactsToFire = 0;
        firedArtifacts = 0;
        droppedFirstArtifact = false;
        reverseSpindexerCase = false;
        spindexer.rotateLeft = false;  // NEW
        spindexer.rotateRight = false; // NEW
        spindexer.resetSlots();
        spindexer.rotateToSlot(0);
        setState(State.IDLE);
    }
    
    public double getAngleToGoal() {
        return follower.getCurrentPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
    }
}
