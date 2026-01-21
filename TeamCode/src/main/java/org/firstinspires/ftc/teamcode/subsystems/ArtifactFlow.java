package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;


public class ArtifactFlow {
    public Spindexer2 spindexer;
    public Paddles paddles;
    public Ramp ramp;
    public Intake intake;
    public Flywheel flywheel;
    
    public ArtifactDetector rightColorSensor;
    public ArtifactDetector leftColorSensor;
    
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    
    public Artifact incomingArtifact = Artifact.NONE;
    
    public int firedArtifacts = 0;
    
    ElapsedTime debounceTimer = new ElapsedTime();
    
    public enum FlowState {
        IDLE,
        INTAKING,
        REVVING,
        LAUNCHING
    }
    
    FlowState state = FlowState.IDLE;
    
    public ArtifactFlow(HardwareMap hardwareMap) {
        this.spindexer = new Spindexer2(hardwareMap);
        this.paddles = new Paddles(hardwareMap);
        this.ramp = new Ramp(hardwareMap);
        this.intake = new Intake(hardwareMap);
        this.flywheel = new Flywheel(hardwareMap);
        rightColorSensor = new ArtifactDetector(hardwareMap, "rightColorSensor");
        leftColorSensor = new ArtifactDetector(hardwareMap, "leftColorSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
    }
    
    public Artifact getDetectedIncomingArtifact() {
        Artifact detected = rightColorSensor.getDetectedArtifact();
        if (detected == Artifact.NONE) {
            return leftColorSensor.getDetectedArtifact();
        }
        return detected;
    }
    
    public boolean artifactIsInSpindexer() {
        return leftDistanceSensor.getDistance(DistanceUnit.INCH) < 3
            || rightDistanceSensor.getDistance(DistanceUnit.INCH) < 3;
    }
    
    public void update() {
        switch (state) {
            case IDLE:
                intake.stop();
                if (ramp.isStationary()) {
                    paddles.open();
                }
                if (paddles.isStationary()) {
                    ramp.loadToSpindexer();
                }
                flywheel.stop();
                firedArtifacts = 0;
                break;
            case REVVING:
                flywheel.setRpmFromDistance();
                break;
            case INTAKING:
                intake();
                if (spindexer.getNumberOfArtifacts() == 3) {
                    state = FlowState.REVVING;
                    return;
                }
                break;
            case LAUNCHING:
                launch();
                if (flywheel.isAtSpeed() && spindexer.getNumberOfArtifacts() == 0) {
                    firedArtifacts = 0;
                    spindexer.rotateToSlot(0);
                    state = FlowState.IDLE;
                    return;
                }
                break;
        }
    }
//
//    private void revLauncher() {
//        flywheel.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.localizer.getPose().getPosition()));
//    }
//
//    public void revTowardGoal() {
//        revLauncher();
//        follower.setLockedHeading(getAngleToGoal());
//    }
    
//    public double getAngleToGoal() {
//        return follower.localizer.getPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
//    }
    
    public void intake() {
        Artifact detectedArtifact = getDetectedIncomingArtifact();
        boolean artifactIsInSpindexer = artifactIsInSpindexer();
        int count = spindexer.getNumberOfArtifacts();
        
        if (detectedArtifact.isArtifact() && !incomingArtifact.isArtifact()) {
            incomingArtifact = detectedArtifact;
            spindexer.artifacts[count] = incomingArtifact;
            
            if (spindexer.getNumberOfArtifacts() == 3 && ramp.isStationary()) {
                paddles.close(); // push last ball up
            }
        }
        
        if (incomingArtifact.isArtifact() && artifactIsInSpindexer && !spindexer.isRotating()) {
            switch (count) {
                case 0:
                    spindexer.rotateToSlot(1);
                    break;
                case 1:
                    spindexer.rotateToSlot(2);
                    break;
                case 2:
                    // in auto, if pattern is PGP then put 2 purples next to each other when
                    //  && !Artifact.patternIsPGP(Robot.motifPattern)
                    if (spindexer.artifacts[1] == incomingArtifact) { // !!?
                        spindexer.rotateToSlot(2.5);
                    }
                    else {
                        spindexer.rotateToSlot(1.5);
                    }
            }
            incomingArtifact = Artifact.NONE;
        }
    }

    public void launch() {
        Artifact motifArtifact = Robot.motifPattern[firedArtifacts];
        
        if (!artifactIsInSpindexer() && !spindexer.isRotating() && spindexer.getNumberOfArtifacts() > 0) {
            spindexer.artifacts[spindexer.getNumberOfArtifacts() - 1] = Artifact.NONE;
        }
        
        if (flywheel.isAtSpeed()) {
            spindexer.rotateToSlot(spindexer.findBestRotationToArtifact(motifArtifact));
        }
        
        if (getDetectedIncomingArtifact().isArtifact()) {
            intake.setTargetPower(0.2);
        }
        else {
            intake.stop();
        }
        
        if (paddles.isStationary()) {
            ramp.feedFromSpindexer();
        }
        if (ramp.isStationary()) {
            paddles.open();
        }
    }
}
