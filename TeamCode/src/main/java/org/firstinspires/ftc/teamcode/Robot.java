package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Paddles;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.Arrays;

public class Robot {
    public Intake intake;
    public Flywheel launcher;
    public Spindexer spindexer;
    public Follower follower;
    public Ramp intakeRamp;
    public Paddles paddles;
    
    public AllianceColor allianceColor = AllianceColor.BLUE;
    public Artifact[] motifPattern = new Artifact[]
        {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};

    boolean spindexerHasRotated = true;
    boolean paddlesRotated = false;
    boolean launchPathClear = true;
    
    int firedArtifacts = 0;
    
    Artifact intakedArtifact = Artifact.NONE;
    
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
        launcher = new Flywheel(hardwareMap);
        
        //launcher.updateVoltComp(13 / follower.getVoltage());
    }

    private void revLauncher() {
        launcher.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition()));
    }
    
    public int artifactsToFire = 0;

    public void update(Telemetry telemetry) {
        follower.update();
        
        telemetry.addData("state", state);
        
        telemetry.addData("current rpm", "%.2f", launcher.getRpm());
        
        telemetry.addData("target rpm", "%.2f", launcher.getTargetRPM());
        
        telemetry.addData("numOfArtifacts", spindexer.getNumberOfArtifacts());
        telemetry.addData("artifacts", Arrays.deepToString(spindexer.slots));
        telemetry.addData("spindexer index", spindexer.currentSlotIndex);
        telemetry.update();
        
        switch (state) {
            case GROUND_FIRE:
                stateTimer.resume();
                intakeRamp.intakeThrough();
                paddles.open();
                
                revLauncher();
                follower.lockHeadingAt(getAngleToGoal());
                
                if (launcher.isUpToSpeed()) {
                    intake.intake();
                }
                break;
            case LOAD_ARTIFACTS:
                launcher.stop();
                intakeRamp.uptake();
                intake.intake();

                if (spindexerHasRotated) {
                    paddles.open();
                }
                else {
                    intake.stop();
                    paddles.close();
                }
                
                Artifact detectedArtifact = spindexer.getDetectedArtifact();
                
                if (detectedArtifact.isArtifact() && stateTimer.isPaused()) {
                    intakedArtifact = detectedArtifact;
                    spindexerHasRotated = false;
                    stateTimer.resetAndStart();
                    paddles.close();
                }
                
                if (stateTimer.seconds() > 1.3) {
                    spindexerHasRotated = true;
                    paddlesRotated = false;
                    stateTimer.pauseAtZero();
                    intakedArtifact = Artifact.NONE;
                } else if ((stateTimer.seconds() > 0.4) && !paddlesRotated) {
                    int count = spindexer.getNumberOfArtifacts();
                    spindexer.slots[count] = intakedArtifact;
                    
                    paddlesRotated = true;
                    
                    switch (count) {
                        case 0:
                            spindexer.rotateToSlot(1);
                            break;
                        case 1:
                            spindexer.rotateToSlot(2);
                            break;
                        case 2:
                            double targetSlot =
                                (spindexer.slots[1] == intakedArtifact) ? 2.5 : 1.5;
                            spindexer.rotateToSlot(targetSlot);
                            
                            intake.stop();
                            intakeRamp.outtake();
                            spindexerHasRotated = true;
                            paddlesRotated = false;
                            intakedArtifact = Artifact.NONE;
                            setState(Robot.State.IDLE);
                            break;
                    }
                }
                
                break;
            case IDLE:
                stateTimer.resume();
                
                if (stateTimer.seconds() > 0.5) {
                    paddles.open();
                }
                
                launcher.stop();
                intake.stop();
                follower.lockHeadingAt(null);
                break;
            case FIRING: // slow FIRING
                // 2. Detect shot by RPM drop
                if (!launchPathClear && launcher.rpmDropped()) {
                    launchPathClear = true;
                }

                // 3. Once shooter recovers, rotate to next artifact
                if (launchPathClear && launcher.isUpToSpeed()) {
                    if (spindexer.getNumberOfArtifacts() == 0) {
                        spindexer.rotateToSlot(0);
                        setState(State.IDLE);
                        break;
                    }
                    
                    Artifact motifArtifact =
                        motifPattern[3 - spindexer.getNumberOfArtifacts()];
                    if (!spindexer.rotateToArtifact(motifArtifact)) {
                        spindexer.rotateToArtifact(motifArtifact.oppositeColor());
                    }
                    launchPathClear = false;
                }
            case FAST_FIRING: // FAST FIRING
                if (launcher.isUpToSpeed()) {
                    stateTimer.resume();

                    boolean isPGP =
                        Arrays.equals(motifPattern, new Artifact[]{ Artifact.PURPLE, Artifact.GREEN,
                            Artifact.PURPLE });

                    spindexer.forceRotateToArtifact(motifPattern[0]);

                    boolean shouldRun23 = !isPGP || firedArtifacts >= 1;

                    if (shouldRun23) {
                        if (!spindexer.rotateToArtifact(motifPattern[1])) {
                            spindexer.rotateToArtifact(motifPattern[1].oppositeColor());
                        }
                        if (!spindexer.rotateToArtifact(motifPattern[2])) {
                            spindexer.rotateToArtifact(motifPattern[2].oppositeColor());
                        }
                    }
                    
                    spindexer.rotateLeft = false;
                    spindexer.rotateRight = false;
                }

                // 2. Detect shot by RPM drop
                if (!launchPathClear && launcher.rpmDropped()) {
                    launchPathClear = true;
                }

                // 3. Once shooter recovers, rotate to next artifact
                if (launchPathClear && launcher.isUpToSpeed()) {
                    firedArtifacts++;
                    if (artifactsToFire <= 0) {
                        artifactsToFire = 0;
                        firedArtifacts = 0;
                        spindexer.rotateToSlot(0);
                        setState(State.IDLE);
                        break;
                    }
                    artifactsToFire--;
                    launchPathClear = false;
                }

                if (stateTimer.seconds() > 9) { // safety timeout
                    firedArtifacts = 0;
                    artifactsToFire = 0;
                    spindexer.rotateToSlot(0);
                    setState(State.IDLE);
                }
            case REVVING:
                stateTimer.resume();
                
                revLauncher();
                follower.lockHeadingAt(getAngleToGoal());
                intakeRamp.outtake();
                intake.stop();
                
                if (stateTimer.seconds() > 0.5) {
                    paddles.open();
                }
                break;
        }
        
        // Disables the intakeRamp after moving to avoid strain
//        if (stateTimer.seconds() > 1) {
//            intakeRamp.disable();
//        }
//        else {
//            intakeRamp.enable();
//        }
        
        intake.update();

        launcher.update(follower.getMotionState().deltaTime);
    }
    
    public double getAngleToGoal() {
        return follower.getCurrentPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
    }
}
