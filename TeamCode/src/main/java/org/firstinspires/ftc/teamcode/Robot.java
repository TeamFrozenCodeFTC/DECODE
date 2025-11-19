package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Spindexer.Artifact.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.Spindexer.Artifact.PURPLE;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Paddles;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

import java.util.Arrays;

public class Robot {
    public Intake intake;
    public Launcher launcher;
    public Spindexer spindexer;
    public Follower follower;
    public Ramp intakeRamp;
    public Paddles paddles;
    
    public AllianceColor allianceColor = AllianceColor.BLUE;
    public Spindexer.Artifact[] motifPattern = new Spindexer.Artifact[]
        {GREEN, PURPLE, PURPLE};

    boolean spindexerHasRotated = true;
    boolean paddlesRotated = false;
    boolean launchPathClear = true;
    
    int firedArtifacts = 0;
    
    Spindexer.Artifact intakedArtifact = Spindexer.Artifact.NONE;
    
    public Robot.State state = Robot.State.IDLE;
    Timeout stateTimer = new Timeout();

    public enum State {
        GROUND_FIRE,
        LOAD_ARTIFACTS,
        FIRING,
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
        stateTimer.resetAndStart();
        stateTimer.pause();
    }

    public Robot(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        intakeRamp = new Ramp(hardwareMap);
        paddles = new Paddles(hardwareMap);
        launcher = new Launcher(hardwareMap);
    }

    private void revLauncher() {
        launcher.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition()));
    }

    public void update(Telemetry telemetry) {
        follower.update();
        
        telemetry.addData("state", state);
        
        telemetry.addData("current rpm", "%.2f", launcher.getRpm());
        
        telemetry.addData("target rpm", "%.2f", launcher.getTargetRPM());
        
        telemetry.addData("numOfArtifacts", spindexer.getNumberOfArtifacts());
        telemetry.addData("artifacts", Arrays.deepToString(spindexer.slots));
        
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
                
                Spindexer.Artifact detectedArtifact = spindexer.getDetectedArtifact();
 
                // 1. Detect artifact intaked
//                if (detectedArtifact.isArtifact() && intakedArtifact.isNone()) {
//                    intakedArtifact = detectedArtifact;
//                    spindexerHasRotated = false;
//                   // paddlesRotated = false;
//                    stateTimer.resetAndStart();
//                    paddles.close();
//                }
                if (detectedArtifact.isArtifact() && stateTimer.isPaused()) {
                    intakedArtifact = detectedArtifact;
                    spindexerHasRotated = false;
                    // paddlesRotated = false;
                    stateTimer.resetAndStart();
                    paddles.close();
                }
                
                if (stateTimer.seconds() > 1.3) {
                    spindexerHasRotated = true;
                    paddlesRotated = false;
                    stateTimer.pauseAtZero();
                    intakedArtifact = Spindexer.Artifact.NONE;
                    //  || detectedArtifact.isNone()
                } else if ((stateTimer.seconds() > 0.4) && !paddlesRotated) {
                    int count = spindexer.getNumberOfArtifacts();
                    spindexer.slots[count] = intakedArtifact;
                    
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
                            intakedArtifact = Spindexer.Artifact.NONE;
                            setState(Robot.State.IDLE);
                            break;
                    }
                    
                    paddlesRotated = true;
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
//            case FIRING: // slow FIRING
////                // 2. Detect shot by RPM drop
////                if (!launchPathClear && launcher.rpmDropped()) {
////                    launchPathClear = true;
////                }
////
////                // 3. Once shooter recovers, rotate to next artifact
////                if (launchPathClear && launcher.isUpToSpeed()) {
////                    if (spindexer.getNumberOfArtifacts() == 0) {
////                        spindexer.rotateToSlot(0);
////                        setState(State.IDLE);
////                        break;
////                    }
////
////                    Spindexer.Artifact motifArtifact =
////                        motifPattern[3 - spindexer.getNumberOfArtifacts()];
////                    if (!spindexer.rotateToArtifact(motifArtifact)) { // rotate to
////                        // other color so preserve same direction of rotation
////                        spindexer.rotateToArtifact(motifArtifact.oppositeColor());
////                    }
////                    launchPathClear = false;
////                }
//                break;
            case FIRING: // FAST FIRING
                if (launcher.isUpToSpeed()) {
                    boolean isPGP =
                        Arrays.equals(motifPattern, new Spindexer.Artifact[]{ PURPLE, GREEN, PURPLE });
                    
                    spindexer.forceRotateToArtifact(motifPattern[0]);
                    
                    boolean shouldRun23 = !isPGP || firedArtifacts >= 1;
                    
                    if (shouldRun23) {
                        spindexer.forceRotateToArtifact(motifPattern[1]);
                        spindexer.forceRotateToArtifact(motifPattern[2]);
                    }
                }
                
                // 2. Detect shot by RPM drop
                if (!launchPathClear && launcher.rpmDropped()) {
                    launchPathClear = true;
                }
                
                // 3. Once shooter recovers, rotate to next artifact
                if (launchPathClear && launcher.isUpToSpeed()) {
                    firedArtifacts++;
                    if (firedArtifacts >= 3) { // what if we don't have 3 artifacts?
                        firedArtifacts = 0;
                        spindexer.rotateToSlot(0);
                        setState(State.IDLE);
                        break;
                    }
              
                    launchPathClear = false;
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
