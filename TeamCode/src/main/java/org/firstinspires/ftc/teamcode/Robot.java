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
    Launcher shooter;
    Spindexer spindexer;
    Follower follower;
    Ramp intakeRamp;
    Paddles paddles;
    
    AllianceColor allianceColor = AllianceColor.BLUE;

    double distanceToGoal;
    
    public enum State {
        GROUND_FIRE,
        LOAD_ARTIFACTS,
        FIRING,
        REVVING,
        IDLE,
        AUTO_TRANSITION,
    }
    
    Robot.State state = Robot.State.IDLE;
    
    boolean hasRotated = false;
    
    Timeout stateTimer = new Timeout();
    
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
        shooter = new Launcher(hardwareMap);
    }
    
    Spindexer.Artifact detectedArtifact = Spindexer.Artifact.NONE;
    
    private void revLauncher() {
        shooter.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition()));
    }
    
    
    boolean artifactWasDetected;
    public void update() {
        follower.update();
        
        switch (state) {
            case GROUND_FIRE:
                intakeRamp.intakeThrough();
                paddles.open();
                
                revLauncher();
                follower.lockHeadingAt(getAngleToGoal());
                
                if (shooter.isUpToSpeed()) {
                    intake.intake();
                }

                break;
            case LOAD_ARTIFACTS:
                spindexer.enableSensors();
                intakeRamp.uptake();
                intake.intake();
                
                if (stateTimer.isPaused()) {
                    paddles.open();
                }
                else {
                    intake.stop();
                    paddles.close();
                }
                
                if (detectedArtifact.isArtifact() && !artifactWasDetected) {
                    artifactWasDetected = true;
                    stateTimer.reset();
                    paddles.close();
                }
       
                if (stateTimer.seconds() > 1) {
                    stateTimer.pauseAtZero();
                    hasRotated = false;
                } else if (stateTimer.seconds() > 0.4 && !hasRotated) {
                    hasRotated = true;
                    if (spindexer.getNumberOfArtifacts() <= 1) {
                        spindexer.slots[spindexer.currentSlotIndex] = detectedArtifact;
                        spindexer.rotateToSlot(spindexer.currentSlotIndex - 1);
                    }
                    else if (spindexer.getNumberOfArtifacts() == 2) {
                        spindexer.slots[spindexer.currentSlotIndex] = detectedArtifact;
                        if (spindexer.slots[1] == detectedArtifact) {
                            spindexer.partiallyRotate(spindexer.currentSlotIndex - 1);
                        }
                        else {
                            spindexer.partiallyRotate(spindexer.currentSlotIndex);
                        }
                        intake.stop();
                        intakeRamp.outtake();
                        artifactWasDetected = false;
                        setState(Robot.State.IDLE);
                    }
                }
                
                break;
            case IDLE: // holding artifacts
                spindexer.disableSensors();
                stateTimer.resume();
                
                if (stateTimer.seconds() > 0.5) {
                    paddles.open();
                }
                
                shooter.stop();
                intake.stop();
                follower.lockHeadingAt(null);
                break;
                
            case FIRING:
                if (shooter.isUpToSpeed()) { // and is in the zone
                    spindexer.rotateToSlot(2);
                    stateTimer.resume();
                    
                    if (stateTimer.seconds() > 3) {
                        setState(State.IDLE);
                        spindexer.resetSlots();
                    }
                    //spindexer.rotateToSlot(spindexer.currentSlotIndex + 2);
                }
                

            case REVVING:
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

        shooter.update(follower.getMotionState().deltaTime);
    }
    
    public double getAngleToGoal() {
        return follower.getCurrentPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
    }
}
