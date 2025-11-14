package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Paddles;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class Robot {
    Intake intake;
    Launcher launcher;
    Spindexer spindexer;
    Follower follower;
    Ramp intakeRamp;
    Paddles paddles;
    
    AllianceColor allianceColor = AllianceColor.BLUE;
    Spindexer.Artifact[] motifPattern = new Spindexer.Artifact[]
        {Spindexer.Artifact.GREEN, Spindexer.Artifact.PURPLE, Spindexer.Artifact.PURPLE};
    
    int firedArtifacts = 0;
    boolean artifactWasDetected = false;
    boolean spindexerHasRotated = false;
    boolean paddlesRotated = false;
    boolean launchPathClear = true;
    
    Robot.State state = Robot.State.IDLE;
    Timeout stateTimer = new Timeout();
    
    Spindexer.Artifact detectedArtifact = Spindexer.Artifact.NONE;
    
    public enum State {
        GROUND_FIRE,
        LOAD_ARTIFACTS,
        FIRING,
        REVVING,
        IDLE,
        AUTO_TRANSITION,
    }
    
    public void setState(Robot.State newState) {
        if (state == newState) {
            return;
        }
        state = newState;
        stateTimer.reset();
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

    public void update() {
        follower.update();
        spindexer.disableSensors();
        
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
                spindexer.enableSensors();
                intakeRamp.uptake();
                intake.intake();
                
                // todo read color sensors to detect artifact
                
                if (spindexerHasRotated) {
                    paddles.open();
                }
                else {
                    intake.stop();
                    paddles.close();
                }
                
                // 1. Detect artifact intaked
                if (detectedArtifact.isArtifact() && !artifactWasDetected) {
                    artifactWasDetected = true;
                    stateTimer.reset(); // starts timer
                    paddles.close();
                }
       
                if (stateTimer.seconds() > 1) {
                    spindexerHasRotated = true;
                    paddlesRotated = false;
                    artifactWasDetected = false;
                } else if (stateTimer.seconds() > 0.4 && !paddlesRotated) {
                    paddlesRotated = true;
                    
                    int count = spindexer.getNumberOfArtifacts();
                    spindexer.slots[count] = detectedArtifact;
                    
                    switch (count) {
                        case 0:
                            spindexer.rotateToSlot(1);
                            break;
                        case 1:
                            spindexer.rotateToSlot(2);
                            break;
                        case 2:
                            double targetSlot = (spindexer.slots[1] == detectedArtifact) ? 1.5 : 2.5;
                            spindexer.rotateToSlot(targetSlot);
                            
                            intake.stop();
                            intakeRamp.outtake();
                            paddlesRotated = false;
                            artifactWasDetected = false;
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
            case FIRING:
                // 2. Detect shot by RPM drop
                if (!launchPathClear && launcher.rpmDropped()) {
                    launchPathClear = true;
                    if (firedArtifacts >= motifPattern.length) {
                        firedArtifacts = 0;
                        setState(State.IDLE);
                        break;
                    }
                }
                
                // 3. Once shooter recovers, rotate to next artifact
                if (launchPathClear && launcher.isUpToSpeed()) {
                    spindexer.rotateToArtifact(motifPattern[firedArtifacts]);
                    launchPathClear = false;
                    firedArtifacts++;
                }
                
                // 4. Safety timeout
                if (stateTimer.seconds() > 3) {
                    setState(State.IDLE);
                    firedArtifacts = 0;
                    spindexer.resetSlots();
                }
            case REVVING:
                stateTimer.resume();
                
                revLauncher();
                follower.lockHeadingAt(getAngleToGoal());
                intakeRamp.outtake();
                intake.stop();
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
