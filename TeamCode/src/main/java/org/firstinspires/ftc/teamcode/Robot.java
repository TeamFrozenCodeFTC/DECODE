package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public Flywheel launcher;
    public Spindexer spindexer;
    public Follower follower;
    public Ramp intakeRamp;
    public Paddles paddles;
    
    public AllianceColor allianceColor = AllianceColor.BLUE;
    public Artifact[] motifPattern = new Artifact[]
        {Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE};

    boolean spindexerHasRotated = true;
    boolean paddlesRotatedUp = false;
    boolean launchPathClear = true;
    
    int firedArtifacts = 0;
    
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
        FIRING,
        FAST_FIRING,
        REVVING,
        IDLE,
        SALVO,
        AUTO_TRANSITION,
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
        timer2.resetAndStart();
    }

    public Robot(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        intakeRamp = new Ramp(hardwareMap);
        paddles = new Paddles(hardwareMap);
        launcher = new Flywheel(hardwareMap);
        
        launcher.filteredVoltage = follower.getVoltage();
    }

    private void revLauncher() {
        launcher.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition()));
    }
    
    private void revTowardGoal() {
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
    
    Timeout timer2 = new Timeout();

    public void update() {
        follower.update();
        
        switch (state) {
            case GROUND_FIRE:
                stateTimer.resume();
                intakeRamp.intakeThrough();
                paddles.open();
                
                revTowardGoal();
                
                if (launcher.isUpToSpeed()) {
                    intake.intake();
                }
                break;
            case LOAD_ARTIFACTS:
                launcher.stop();
                intakeRamp.uptake();
                intake.intake();
                
                if (spindexerHasRotated) {
                    paddles.open(); // down
                }
                else {
                    intake.stop();
                    paddles.close();
                }
                
                Artifact detectedArtifact = spindexer.getDetectedArtifact();
                boolean artifactPresent = detectedArtifact.isArtifact();
                
                if (artifactPresent) {
                    if (paddlesRotatingDown) {
                        intake.outtake();
                    }
                    
                    if (stateTimer.isPaused()) {
                        intakedArtifact = detectedArtifact;
                        spindexerHasRotated = false;
                        stateTimer.resetAndStart();
                        paddles.close();
                    }
                }
                else {
                    paddlesRotatingDown = false;
                }
                
                if (stateTimer.seconds() > 1.1) { // was 1.3
                    spindexerHasRotated = true;
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
                        spindexerHasRotated = true;
                        paddlesRotatedUp = false;
                        intakedArtifact = Artifact.NONE;
                        updateReverseCase();
                        setState(State.REVVING);
                    }
                }
                
                break;
            case IDLE:
                droppedFirstArtifact = false;
                
                stateTimer.resume();
                
                if (stateTimer.seconds() > 0.5) {
                    paddles.open();
                }
                
                launcher.stop();
                intake.stop();
                follower.lockHeadingAt(null);
                break;
                
//            case FAST_FIRING: // FAST FIRING
//                stateTimer.resume();
//
//                revLauncher();
//                follower.lockHeadingAt(getAngleToGoal());
//                intakeRamp.outtake();
//                intake.stop();
//
//                if (stateTimer.seconds() > 0.5) {
//                    paddles.open();
//                }
//
//                if (launcher.isUpToSpeed()) {
//                    stateTimer.resume();
//
//                    boolean isPGP =
//                        Arrays.equals(motifPattern, new Artifact[]{ Artifact.PURPLE, Artifact.GREEN,
//                            Artifact.PURPLE });
//
//                    boolean shouldRun23 = !isPGP || firedArtifacts >= 1;
//
//                    if (!countedArtifacts) {
//                        hasPGP = Collections.frequency(List.of(spindexer.slots),
//                                                       Artifact.GREEN) == 1 && Collections.frequency(List.of(spindexer.slots),
//                                                                                                     Artifact.PURPLE) == 2;
//                        countedArtifacts = true;
//                    }
//
//                    // if hasPGP then run only once, otherwise run always
//                    if (!rotatedFirst || !hasPGP) {
//                        if (!spindexer.rotateToArtifact(motifPattern[0])) {
//                            spindexer.rotateToArtifact(motifPattern[0].oppositeColor());
//                        }
//                    }
//                    rotatedFirst = true;
//
//                    if (shouldRun23) {
//                        if (!spindexer.rotateToArtifact(motifPattern[1])) {
//                            spindexer.rotateToArtifact(motifPattern[1].oppositeColor());
//                        }
//                        if (!spindexer.rotateToArtifact(motifPattern[2])) {
//                            spindexer.rotateToArtifact(motifPattern[2].oppositeColor());
//                        }
//                    }
//
//                    spindexer.rotateLeft = false;
//                    spindexer.rotateRight = false;
//                }
//
//                // fix artifact getting stuck under paddles
//
//                // 2. Detect shot by RPM drop
//                if (!launchPathClear && launcher.rpmDropped()) {
//                    firedArtifacts++;
//                    launchPathClear = true;
//                }
//                // 3. Once shooter recovers, rotate to next artifact
//                if (launchPathClear && launcher.isUpToSpeed()) {
//                    if (artifactsToFire <= 0) {
//                        artifactsToFire = 0;
//                        firedArtifacts = 0;
//                        rotatedFirst = false;
//                        countedArtifacts = false;
//                        hasPGP = false;
//                        spindexer.resetSlots();
//                        spindexer.rotateToSlot(0);
//                        setState(State.IDLE);
//                        break;
//                    }
//                    artifactsToFire--;
//                    launchPathClear = false;
//                }
//
//                if (stateTimer.seconds() > 5) { // safety timeout
//                    firedArtifacts = 0;
//                    artifactsToFire = 0;
//                    spindexer.resetSlots();
//                    rotatedFirst = false;
//                    countedArtifacts = false;
//                    hasPGP = false;
//                    spindexer.rotateToSlot(0);
//                    setState(State.IDLE);
//                }
//                break;
            case FAST_FIRING:
                timer2.resume();
                
                revTowardGoal();
                intakeRamp.outtake();
                intake.stop();
                
                if (stateTimer.seconds() > 0.5) {
                    paddles.open();
                }
                
                if (launcher.isUpToSpeed()) {
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

                if (droppedFirstArtifact && (launcher.hasDroppedRPM())) {
                    firedArtifacts++;
                    if (artifactsToFire <= 0) {
                        resetSpindexer();
                        break;
                    }
                    artifactsToFire--;
                    timer2.resetAndStart();
                }
                
                if (isAuto && stateTimer.seconds() > 2.5) {
                    resetSpindexer();
                }
                if (!isAuto && stateTimer.seconds() > 6) {
                    resetSpindexer();
                }
                break;
            case FIRING: // slow FIRING
                // 2. Detect shot by RPM drop
                if (!launchPathClear && launcher.hasDroppedRPM()) {
                    firedArtifacts++;
                    launchPathClear = true;
                }

                // 3. Once shooter recovers, rotate to next artifact
                if (launchPathClear && launcher.isUpToSpeed()) {
                    if (spindexer.getNumberOfArtifacts() == 0) {
                        spindexer.rotateToSlot(0);
                        firedArtifacts = 0;
                        setState(State.IDLE);
                        break;
                    }
                    
                    Artifact motifArtifact = motifPattern[firedArtifacts];
                    spindexer.forceRotateToArtifact(motifArtifact);
                    launchPathClear = false;
                }
            case REVVING:
                stateTimer.resume();
                
                revTowardGoal();
                intakeRamp.outtake();
                intake.stop();
                
                if (stateTimer.seconds() > 0.7) { // was .5
                    paddles.open();
                }
                break;
        }
        
        intake.update();

        launcher.update(follower.getMotionState().deltaTime, follower.getVoltage());
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
