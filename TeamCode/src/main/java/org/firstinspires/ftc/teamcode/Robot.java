package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRamp;
import org.firstinspires.ftc.teamcode.subsystems.Paddles;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot {
    Intake intake;
    Shooter shooter;
    Spindexer spindexer;
    Follower follower;
    IntakeRamp intakeRamp;
    Paddles paddles;
    
    AllianceColor allianceColor;

    boolean isIntaking = false;
    boolean isShooting = false;
    boolean artifactBeingShot = false;

    double distanceToGoal;
    
//    public enum State {
//        INTAKING,
//        SHOOTING,
//        IDLE
//    }
//
//    State state;
    
    boolean isRevingToGoal = false;
    
    public Robot(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intakeRamp = new IntakeRamp(hardwareMap);
        paddles = new Paddles(hardwareMap);
        
        double voltageCompensation = follower.drivePowerController.getVoltagePowerCompensation();
        
        //shooter.updateFeedforwardByVoltage(voltageCompensation);
    }
    
    public void revUpShooterBasedOnDistance() {
        isRevingToGoal = true;
    }
    
    public void throughTake() {
        intakeRamp.intakeThrough();
        paddles.open();
    }
    
    public void uptake() {
        intakeRamp.uptake();
        paddles.open();
    }
    
    public void update() {
        shooter.update(follower.getMotionState().deltaTime);
        
        distanceToGoal =
            allianceColor.getGoalPosition().distanceTo(follower.getCurrentPose().getPosition());
        
        if (isRevingToGoal) {
            shooter.setRpmFromDistance(distanceToGoal);
        }

//
//        if (requestLaunch) {
//            shooter.setRpmFromDistance(distanceToGoal);
//        }
//
//        switch (state) {
//            case IDLE:
//                if (indexer.isAtMaxCapacity()) {
//                    indexer.lockArtifacts();
//                    shooter.setRpmFromDistance(distanceToGoal);
//                }
//                break;
//            case INTAKING:
//                intake.intake();
//                shooter.uptake();
//                if (indexer.isAtMaxCapacity()) {
//                    indexer.lockArtifacts();
//                    stopIntake();
//                    shooter.setRpmFromDistance(distanceToGoal);
//                    state = State.IDLE;
//                }
//                break;
//        }
//
//        if (isIntaking && indexer.artifactIsDetected() && !indexer.isAtMaxCapacity()) {
//            indexer.rotateToNextEmptySlot();
//        }
//        if (indexer.isAtMaxCapacity()) {
//            indexer.lockArtifacts();
//            stopIntake();
//            shooter.setRPM(3500);
//        }
//
//        if (isReadyToShoot() && !artifactBeingShot) {
//            indexer.rotateToArtifact(); // drops artifact
//            intake.spinIntoShooter(); // spins intake to help feed artifact into shooter
//            artifactBeingShot = true;
//        }
//        if (artifactBeingShot && !shooter.isUpToSpeed()) { // artifact has touched the
//            // shooter (might cause problems if shooter didn't slow down)
//            artifactBeingShot = false;
//            indexer.rotatePartiallyToArtifact(); // rotates to next artifact without
//            // dropping it
//        }
//
//        if (indexer.getNumberOfArtifacts() <= 0 && !artifactBeingShot && shooter.isUpToSpeed()) {
//            shooter.stop();
//            intake.stop();
//            isShooting = false;
//        }
        
        follower.update();
    }
    
    public void shootArtifacts() {
        isShooting = true;
    }
    
    public boolean isReadyToShoot() {
        return shooter.isUpToSpeed() && isShooting && spindexer.getNumberOfArtifacts() > 0;
    }
    
    public void intake() {
        intake.intake();
        shooter.uptake();
        isIntaking = true;
    }
    
    public void stopIntake() {
        intake.stop();
        shooter.stop();
        isIntaking = false;
    }

    public double getAngleToGoal() {
        return follower.getCurrentPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
    }
}
