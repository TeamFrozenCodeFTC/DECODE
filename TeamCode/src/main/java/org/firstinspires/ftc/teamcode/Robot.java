package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.spindexer.MotifPattern;
import org.firstinspires.ftc.teamcode.subsystems.spindexer.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Timeout;

public class Robot {
    // robot is 17 inches long, 16.5 wide
    
    public static AllianceColor allianceColor = AllianceColor.BLUE;
    public static MotifPattern motifPattern = MotifPattern.GPP;
    public static Pose currentPose;
    
    public static double currentSpindexerIndex = 0;
    
    public static Artifact[] artifacts = null;
    public Spindexer spindexer;
    public Transfer transfer;
    public Intake intake;
    public Flywheel flywheel;
    public Artifact incomingArtifact = Artifact.NONE;
    public int firedArtifacts = 0;
    public Follower follower;
    public boolean isAuto = false;
    
    public Robot.State state = Robot.State.IDLE;
    public Timeout stateTimer = new Timeout();
    
    // Prevents quick misfires
    public Timeout intakeTimer = new Timeout();
    Timeout firingTimer = new Timeout();
    
    boolean dropping = true;
    
    public Robot(HardwareMap hardwareMap) {
        follower = FollowerConstants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
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
    
    private void revLauncher() {
        flywheel.setRpmFromDistance(allianceColor.getGoalPosition()
                                        .distanceTo(
                                            follower.localizer.getPose().getPosition()));
    }
    
    public void revTowardGoal() {
        revLauncher();
        follower.setLockedHeading(getAngleToGoal());
    }
    
    public boolean isLookingAtGoal() {
        return Math.abs(getAngleToGoal() - follower.localizer.getPose().getHeading()) <
            Math.toRadians(2.5);
    }
    
    public void intake() {
        flywheel.stop();
        transfer.loadToSpindexer();
        intake.intake();
        stateTimer.resume();
        
        Artifact detectedArtifact = spindexer.getDetectedArtifact();
        
        //  && spindexer.isSpindexerClear()
        if (detectedArtifact.isArtifact() && !incomingArtifact.isArtifact()) {
            incomingArtifact = detectedArtifact;
            spindexer.artifacts[Spindexer.rollIndex((int) spindexer.currentSlotIndex)] =
                incomingArtifact;
            intakeTimer.resetAndStart();
        }
        
        if (incomingArtifact.isArtifact() && spindexer.isArtifactInHandoffZone() && intakeTimer.seconds() > 0.5) {
            spindexer.intakeArtifact(incomingArtifact, motifPattern);
            incomingArtifact = Artifact.NONE;
        }
    }
    
    public void launch() {
        if (firedArtifacts >= Robot.motifPattern.getPattern().length) return;
        
        Artifact motifArtifact = Robot.motifPattern.getPattern()[firedArtifacts];
        
        if (flywheel.isAtSpeed() && (isAuto || isLookingAtGoal())) {
            spindexer.rotateAndDrop(spindexer.findBestRotationToArtifact(motifArtifact));
        }
        
        if (spindexer.didArtifactJustDrop()) {
            spindexer.artifacts[Spindexer.rollIndex((int) spindexer.currentSlotIndex)] =
                Artifact.NONE;
            firedArtifacts++;
            //dropping = false;
        }
        
        transfer.feedFromSpindexer();
        transfer.openPaddles();
    }
    
    public void update() {
        follower.update();
        flywheel.readSensors();
        
        switch (state) {
            case IDLE:
                intakeExcessArtifacts();
                
                flywheel.stop();
                if (intake.getTargetPower() >= 0) {
                    intake.stop();
                }
                follower.setLockedHeading(null);
                break;
            case FIRE_THROUGH:
                stateTimer.resume();
                transfer.feedFromIntake();
                transfer.openPaddles();
                
                revTowardGoal();
                
                if (flywheel.isAtSpeed()) {
                    intake.intake();
                }
                break;
            case LAUNCHING:
                revTowardGoal();
                launch();
                if (flywheel.isAtSpeed() && spindexer.getNumberOfArtifacts() == 0 && firingTimer.seconds() > 0.1) {
                    firedArtifacts = 0;
                    spindexer.reset();
                    spindexer.rotateToSlot(0);
                    setState(State.IDLE);
                    return;
                }
                break;
            case REVVING:
                intakeExcessArtifacts();
                revTowardGoal();
                if (!incomingArtifact.isArtifact()) {
                    transfer.feedFromSpindexer();
                }
                intake.stop();
                break;
            case INTAKING:
                intake();
                if (spindexer.getNumberOfArtifacts() == 3) {
                    setState(State.REVVING);
                    return;
                }
                break;
        }
        
        transfer.update();
        intake.update(follower.deltaTime);
        flywheel.update(follower.deltaTime, follower.getVoltage());

        saveData();
    }
    
    public void saveData() {
        Robot.artifacts = spindexer.artifacts;
        Robot.currentPose = follower.localizer.getPose();
        Robot.currentSpindexerIndex = spindexer.currentSlotIndex;
    }
    
    public void intakeExcessArtifacts() {
        if (incomingArtifact.isArtifact() && spindexer.isSpindexerClear()) {
            transfer.closePaddles();
        }
        
        if (
            incomingArtifact.isArtifact()
                && transfer.getPaddleMode() == Transfer.PaddleMode.CLOSED
                && spindexer.isArtifactInHandoffZone() && intakeTimer.seconds() > 0.5) {
            spindexer.intakeArtifact(incomingArtifact, motifPattern);
            incomingArtifact = Artifact.NONE;
            stateTimer.resume();
        }
        
        if (stateTimer.seconds() > 0.5) {
            transfer.openPaddles();
        }
    }
    
    public void resetSpindexer() {
        firedArtifacts = 0;
        spindexer.reset();
        spindexer.rotateToSlot(0);
        setState(State.IDLE);
    }
    
    public double getAngleToGoal() {
        return follower.localizer.getPose()
            .getPosition()
            .getAngleToLookAt(allianceColor.getGoalPosition());
    }
    
    public enum State {
        IDLE,
        INTAKING,
        REVVING,
        FIRE_THROUGH,
        LAUNCHING
    }
}
