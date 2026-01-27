package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Paddles;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
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
    public Paddles paddles;
    public Ramp ramp;
    public Intake intake;
    public Flywheel flywheel;
    public Artifact incomingArtifact = Artifact.NONE;
    public int firedArtifacts = 0;
    public Follower follower;
    public boolean isAuto = false;
    
    public Robot.State state = Robot.State.IDLE;
    Timeout stateTimer = new Timeout();
    Timeout intakeTimer = new Timeout();
    
    public Robot(HardwareMap hardwareMap) {
        follower = FollowerConstants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        ramp = new Ramp(hardwareMap);
        paddles = new Paddles(hardwareMap);
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
        ramp.loadToSpindexer();
        intake.intake();
        stateTimer.resume();
        
        Artifact detectedArtifact = spindexer.getDetectedArtifact();
        
        if (detectedArtifact.isArtifact() && !incomingArtifact.isArtifact()) {
            incomingArtifact = detectedArtifact;
            spindexer.artifacts[Spindexer.rollIndex((int) spindexer.currentSlotIndex)] =
                incomingArtifact;
            intakeTimer.resetAndStart();
        }
        
        if (incomingArtifact.isArtifact() && spindexer.didArtifactJustEnterSpindexer() &&
            intakeTimer.seconds() > 0.25) {
            spindexer.intakeArtifact(incomingArtifact, motifPattern);
            incomingArtifact = Artifact.NONE;
        }
    }
    
    public void launch() {
        if (firedArtifacts >= Robot.motifPattern.getPattern().length) return;
        
        Artifact motifArtifact = Robot.motifPattern.getPattern()[firedArtifacts];
        
        if (spindexer.didArtifactJustDrop()) {
            spindexer.artifacts[Spindexer.rollIndex((int) spindexer.currentSlotIndex)] =
                Artifact.NONE;
            firedArtifacts++;
        }
        
        if (flywheel.isAtSpeed() && (isAuto || isLookingAtGoal())) {
            spindexer.rotateToSlot(spindexer.findBestRotationToArtifact(motifArtifact));
        }
        
        if (paddles.isStationary()) {
            ramp.feedFromSpindexer();
        }
        if (ramp.isStationary()) {
            paddles.open();
        }
    }
    
    public void update() {
        follower.update();
        flywheel.readSensors();
        paddles.update();
        ramp.update();
        
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
                ramp.feedFromIntake();
                paddles.open();
                
                revTowardGoal();
                
                if (flywheel.isAtSpeed()) {
                    intake.intake();
                }
                break;
            case LAUNCHING:
                revTowardGoal();
                launch();
                if (flywheel.isAtSpeed() && spindexer.getNumberOfArtifacts() == 0) {
                    firedArtifacts = 0;
                    spindexer.reset();
                    spindexer.rotateToSlot(0);
                    setState(State.IDLE);
                    return;
                }
                break;
            case REVVING:
                revTowardGoal();
                
                intakeExcessArtifacts();
                if (!isAuto && paddles.isStationary()) {
                    ramp.feedFromSpindexer();
                }
                
                intake.stop();

                break;
            case INTAKING:
                intake();
                if (spindexer.getNumberOfArtifacts() == 3) {
                    state = State.REVVING;
                    return;
                }
                break;
        }
        
        intake.update(follower.deltaTime);
        flywheel.update(follower.deltaTime, follower.getVoltage());
        
        Robot.artifacts = spindexer.artifacts;
        Robot.currentPose = follower.localizer.getPose();
        Robot.currentSpindexerIndex = spindexer.currentSlotIndex;
    }
    
    public void intakeExcessArtifacts() {
        if (incomingArtifact.isArtifact()) {
            if (ramp.isStationary()) {
                paddles.close();
            }
            if (spindexer.didArtifactJustEnterSpindexer()) {
                spindexer.intakeArtifact(incomingArtifact, motifPattern);
                incomingArtifact = Artifact.NONE;
                stateTimer.resume();
            }
        }
        
        if (stateTimer.seconds() > 0.5) {
            paddles.open();
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
