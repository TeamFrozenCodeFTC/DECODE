package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Paddles;
import org.firstinspires.ftc.teamcode.utils.Timeout;

public class Robot {
    // robot is 17 inches long, 16.5 wide
    
    public Intake intake;
    public Flywheel flywheel;
    public Spindexer spindexer;
    public Follower follower;
    public Ramp intakeRamp;
    public Paddles paddles;
    
    public static AllianceColor allianceColor = AllianceColor.BLUE;
    public static Artifact[] motifPattern = new Artifact[]
        {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
    public static Pose currentPose;
    public static Artifact[] artifacts = null;
    
    public int firedArtifacts = 0;
    public Integer firingAllIndex = null;
    
    public boolean spindexerIsRotating = false;
    
    public boolean isAuto = false;
    
    public Artifact intakedArtifact = Artifact.NONE;
    
    public Robot.State state = Robot.State.IDLE;
    Timeout stateTimer = new Timeout();

    public enum State {
        GROUND_FIRE,
        PADDLE_INTAKE,
        MOTIF_FIRING,
        REVVING,
        IDLE,
        FIRING,
        CONTINUOUS_INTAKE,
        STUFF_INTAKE
    }
    
    public void preload(Artifact[] artifacts) {
        spindexer.artifacts = artifacts;
        firedArtifacts = 0;
    }
    
    public void setState(Robot.State newState) {
        if (state == newState) {
            return;
        }
        state = newState;
        stateTimer.pauseAtZero();
    }

    public Robot(HardwareMap hardwareMap) {
        //follower = new Follower(hardwareMap);
        follower = FollowerConstants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        intakeRamp = new Ramp(hardwareMap);
        paddles = new Paddles(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        
        flywheel.filteredVoltage = follower.getVoltage();
    }

    private void revLauncher() {
        flywheel.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.localizer.getPose().getPosition()));
    }
    
    public void revTowardGoal() {
        revLauncher();
        follower.setLockedHeading(getAngleToGoal());
        //follower.lockHeadingAt(getAngleToGoal());
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
            
            boolean isFull = spindexer.intakeArtifact(intakedArtifact);
            
            if (isFull) {
                intake.stop();
                intakeRamp.outtake();
                spindexerIsRotating = false;
                intakedArtifact = Artifact.NONE;
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
            spindexer.artifacts[droppedIndex] =
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
            
            Integer target = spindexer.chooseRotationTarget(motifPattern[firedArtifacts]);
            if (target == null) {
                spindexer.rotateToArtifact(motifPattern[firedArtifacts].oppositeColor());
            }
            else {
                spindexer.rotateToSlot(target);
            }
        }
        
        stateTimer.resume();
        
        revTowardGoal();
        intakeRamp.outtake();
        intake.stop();
        
        if (stateTimer.seconds() > 0.7) { // was .5
            paddles.open();
        }
    }
    
    public void stuffIntake() {
        flywheel.stop();
        intakeRamp.uptake();
        intake.intake();
        
        Artifact detectedArtifact = spindexer.getDetectedArtifact();
        boolean artifactPresent = detectedArtifact.isArtifact();
        
        if (artifactPresent && spindexer.getNumberOfArtifacts() == 0) {
            paddles.close();
        }
        if (spindexer.artifactIsInSpindexer() && spindexer.getNumberOfArtifacts() == 0) {
            spindexer.intakeArtifact(detectedArtifact);
        }
    }
    
    public void firing() {
        if (firingAllIndex == null) {
            int leftIndex = spindexer.shiftLeft(spindexer.currentSlotIndex, 3);
            int rightIndex = spindexer.shiftRight(spindexer.currentSlotIndex, 3);
            
            if (Math.abs(leftIndex) < Math.abs(rightIndex)) {
                firingAllIndex = leftIndex;
            } else {
                firingAllIndex = rightIndex;
            }
        }
        
        if (flywheel.artifactLaunched()) {
            spindexer.artifacts[spindexer.getNumberOfArtifacts() - 1] = Artifact.NONE;
        }
        
        if (flywheel.isUpToSpeed()) {
            spindexer.rotateToSlot(firingAllIndex);
            
            if (spindexer.getNumberOfArtifacts() == 0) {
                resetSpindexer();
                return;
            }
        }
        
        revTowardGoal();
        intakeRamp.outtake();
        intake.stop();
        
        stateTimer.resume();
        
        if (stateTimer.seconds() > 0.7) { // was .5
            paddles.open();
        }
    }
    
    public void revving() {
        revTowardGoal();
        intakeRamp.outtake();
        intake.stop();
        
        stateTimer.resume();
        
        if (stateTimer.seconds() > 0.7) { // was .5
            paddles.open();
        }
        
        if (flywheel.artifactLaunched()) {
            int droppedIndex = Spindexer.rollIndex((int) spindexer.currentSlotIndex);
            spindexer.artifacts[droppedIndex] =
                Artifact.NONE;
            firedArtifacts++;
        }
        
        if (flywheel.isUpToSpeed() && spindexer.getNumberOfArtifacts() == 0) {
            resetSpindexer();
            setState(State.IDLE);
        }
    }
    
    public void continuousIntake() {
        flywheel.stop();
        intakeRamp.uptake();
        intake.intake();
        
        Artifact detectedArtifact2 = spindexer.getDetectedArtifact();
        
        
        boolean artifactIsInSpindexer = spindexer.artifactIsInSpindexer();
        
        if (detectedArtifact2.isArtifact()) {
            if (!intakedArtifact.isArtifact()) {
                intakedArtifact = detectedArtifact2;
            }
            
            if (spindexer.getNumberOfArtifacts() == 2 && !artifactIsInSpindexer) {
                paddles.close();
            }
        }
        
        // Rotates spindexer to intake the artifact that was pushed up.
        //                if (spindexer.artifactIsInSpindexer() && intakedArtifact.isArtifact() && timer.seconds() > 0.5) {
        //                    spindexer.intakeArtifact(intakedArtifact);
        //                    intakedArtifact = Artifact.NONE;
        //                    timer.resetAndStart();
        //
        //                    if (spindexer.getNumberOfArtifacts() == 3) {
        //                        setState(State.IDLE);
        //                    }
        //                }
        
        if (intakedArtifact.isArtifact() && !spindexerIsRotating && artifactIsInSpindexer) {
            spindexerIsRotating = true;
            spindexer.intakeArtifact(intakedArtifact);
            intakedArtifact = Artifact.NONE;
            
            if (spindexer.getNumberOfArtifacts() == 3) {
                setState(State.REVVING);
            }
        }
        else if (spindexerIsRotating && !artifactIsInSpindexer) {
            spindexerIsRotating = false;
        }
    }
    
    public void update() {
        follower.update();
        
        switch (state) {
            case IDLE:
                stateTimer.resume();
                
                if (stateTimer.seconds() > 0.5) {
                    paddles.open();
                }
                
                flywheel.stop();
                intake.stop();
                //follower.lockHeadingAt(null);
                follower.setLockedHeading(null);
                break;
            case GROUND_FIRE:
                fireThrough();
                break;
            case PADDLE_INTAKE:
                sensorLoadArtifacts();
                break;
            case MOTIF_FIRING:
                motifFire();
                break;
            case STUFF_INTAKE: // Loads 1st with paddles and 2nd two in the intake
                stuffIntake();
                break;
            case FIRING:
                firing();
                break;
            case REVVING:
                revving();
                break;
            case CONTINUOUS_INTAKE:
                continuousIntake();
                break;
        }
        
//        intake.update(follower.localizer.deltaTime);
        double deltaTime = follower.deltaTime;
        intake.update(deltaTime);

        flywheel.update(deltaTime, follower.getVoltage());
        
        Robot.artifacts = spindexer.artifacts;
        Robot.currentPose = follower.localizer.getPose();
    }
    
    public void resetSpindexer() {
        firedArtifacts = 0;
        firingAllIndex = null;
        spindexer.rotateLeft = false;  // NEW
        spindexer.rotateRight = false; // NEW
        spindexer.resetSlots();
        spindexer.rotateToSlot(0);
        setState(State.IDLE);
    }
    
    public double getAngleToGoal() {
        return follower.localizer.getPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
    }
}
