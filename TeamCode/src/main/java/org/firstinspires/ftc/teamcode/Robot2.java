package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Paddles;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class Robot2 {
    Intake intake;
    Launcher shooter;
    Spindexer spindexer;
    Follower follower;
    Ramp intakeRamp;
    Paddles paddles;
    
    AllianceColor allianceColor;

    double distanceToGoal;
    
    public enum State {
        GROUND_FIRE,
        LOAD_ARTIFACTS,
        FIRING,
        REVVING,
        IDLE,
        AUTO_TRANSITION,
    }
    
    Robot2.State state = Robot2.State.IDLE;
    
    boolean hasRotated = false;
    
    Timeout stateTimer = new Timeout();
    
    public void setState(Robot2.State newState) {
        state = newState;
        stateTimer.reset();
        stateTimer.pause();
    }

    public Robot2(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Launcher(hardwareMap);
        intakeRamp = new Ramp(hardwareMap);
        paddles = new Paddles(hardwareMap);
        
        double voltageCompensation = follower.drivePowerController.getVoltagePowerCompensation();
        //shooter.updateFeedforwardByVoltage(voltageCompensation);
    }
    
    boolean detectedArtifact = false;
    
    public void update() {
        follower.update();
        
        distanceToGoal =
            allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition());
       
        switch (state) {
            case GROUND_FIRE:
                intakeRamp.intakeThrough();
                paddles.open();
                
                shooter.setRpmFromDistance(distanceToGoal);
                follower.lockHeadingAt(getAngleToGoal());
                
                if (shooter.isUpToSpeed()) {
                    intake.intake();
                }

                break;
            case LOAD_ARTIFACTS:
                intakeRamp.uptake();
                intake.intake();
                
                if (detectedArtifact) {
                    stateTimer.reset();
                    paddles.close();
                } else if (stateTimer.seconds() > 1) {
                    stateTimer.pauseAtZero();
                    hasRotated = false;
                } else if (stateTimer.seconds() > 0.4 && !hasRotated) {
                    hasRotated = true;
                    if (spindexer.currentSlotIndex == 0) {
                        spindexer.partiallyRotate(-0.5);
                        intake.stop();
                        intakeRamp.outtake();
                        // setState(Robot2.State.IDLE);
                    }
                    else {
                        spindexer.rotateToSlot(spindexer.currentSlotIndex - 1);
                    }
                }
                
                if (stateTimer.isPaused()) {
                    paddles.open();
                }
                else {
                    paddles.close();
                }
                
                break;
            case IDLE: // holding artifacts
                if (stateTimer.seconds() > 0.5) {
                    paddles.open();
                }
                
                //intakeRamp.outtake();
                
                shooter.stop();
                intake.stop();
                follower.lockHeadingAt(null);
                break;
                
            case FIRING:
                if (shooter.isUpToSpeed()) { // and is in the zone
                    spindexer.rotateToSlot(2);
                    //                    spindexer.slots[spindexer.currentSlotIndex] =
                    //                        Spindexer.Artifact.NONE;
                }
                
                // Wait 0.5s for the artifact to leave
                //                if (!stateTimer.isPaused() && shooter.getRpm() / shooter.getTargetRPM() < 0.95) {
                //                    stateTimer.resume();
                //                }
            case REVVING:
                shooter.setRpmFromDistance(distanceToGoal);
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
//
        shooter.update(follower.getMotionState().deltaTime);
    }
    
    public double getAngleToGoal() {
        return follower.getCurrentPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
    }
}
