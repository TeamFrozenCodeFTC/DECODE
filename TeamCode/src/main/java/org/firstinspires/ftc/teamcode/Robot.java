package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class Robot {
    Intake intake;
    Shooter shooter;
    Indexer indexer;
    Follower follower;
    
    boolean isIntaking = false;
    boolean isShooting = false;
    boolean artifactBeingShot = false;
    
    public Robot(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        
        double voltageCompensation = follower.drivePowerController.getVoltagePowerCompensation();
        //shooter.updateFeedforwardByVoltage(voltageCompensation);
    }
    
    public void update() {
        shooter.update(follower.getMotionState().deltaTime);
        
        if (isIntaking && indexer.artifactIsDetected() && !indexer.isAtMaxCapacity()) {
            indexer.rotateToNextEmptySlot();
        }
        if (indexer.isAtMaxCapacity()) {
            indexer.lockArtifacts();
            stopIntake();
            shooter.setRPM(5000);
        }
        
        if (isReadyToShoot() && !artifactBeingShot) {
            indexer.rotateToArtifact(); // drops artifact
            intake.spinIntoShooter(); // spins intake to help feed artifact into shooter
            artifactBeingShot = true;
        }
        if (artifactBeingShot && !shooter.isUpToSpeed()) { // artifact has touched the
            // shooter (might cause problems if shooter didn't slow down)
            artifactBeingShot = false;
            indexer.rotatePartiallyToArtifact(); // rotates to next artifact without
            // dropping it
        }
        
        if (indexer.getNumberOfArtifacts() <= 0 && !artifactBeingShot && shooter.isUpToSpeed()) {
            shooter.stop();
            intake.stop();
            isShooting = false;
        }
        
        follower.update();
    }
    
    public void shootArtifacts() {
        isShooting = true;
    }
    
    public boolean isReadyToShoot() {
        return shooter.isUpToSpeed() && isShooting && indexer.getNumberOfArtifacts() > 0;
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

    private static final Vector goal = new Vector(144-6, 144-6);
    
    public double getAngleToGoal() {
        return Math.toDegrees(follower.getCurrentPose().getPosition().getAngleToLookAt(goal));
    }
}
