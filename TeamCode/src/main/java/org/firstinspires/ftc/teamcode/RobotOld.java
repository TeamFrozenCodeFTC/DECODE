//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
//import org.firstinspires.ftc.teamcode.blackice.core.Follower;
//import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
//import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
//import org.firstinspires.ftc.teamcode.subsystems.spindexer.Spindexer;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Ramp;
//import org.firstinspires.ftc.teamcode.subsystems.Paddles;
//import org.firstinspires.ftc.teamcode.utils.Timeout;
//
//public class Robot {
//    // robot is 17 inches long, 16.5 wide
//
//    public Intake intake;
//    public Flywheel flywheel;
//    public Spindexer spindexer;
//    public Follower follower;
//    public Ramp intakeRamp;
//    public Paddles paddles;
//
//    public static AllianceColor allianceColor = AllianceColor.BLUE;
//    public static Artifact[] motifPattern = new Artifact[]
//        {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
//    public static Pose currentPose;
//    public static Artifact[] artifacts = null;
//
//    public int firedArtifacts = 0;
//    public Integer firingAllIndex = null;
//
//    public boolean spindexerIsRotating = false;
//
//    public boolean isAuto = false;
//
//    public Artifact intakedArtifact = Artifact.NONE;
//
//    public Robot.State state = Robot.State.IDLE;
//    Timeout stateTimer = new Timeout();
//
//    public enum State {
//        GROUND_FIRE,
//        PADDLE_INTAKE,
//        MOTIF_FIRING,
//        REVVING,
//        IDLE,
//        FIRING,
//        CONTINUOUS_INTAKE,
//        STUFF_INTAKE,
//        AUTO_FIRE
//    }
//
//    public void preload(Artifact[] artifacts) {
//        spindexer.artifacts = artifacts;
//        firedArtifacts = 0;
//    }
//
//    public void setState(Robot.State newState) {
//        if (state == newState) {
//            return;
//        }
//        state = newState;
//        stateTimer.pauseAtZero();
//    }
//
//    public Robot(HardwareMap hardwareMap) {
//        follower = FollowerConstants.createFollower(hardwareMap);
//        intake = new Intake(hardwareMap);
//        spindexer = new Spindexer(hardwareMap);
//        intakeRamp = new Ramp(hardwareMap);
//        paddles = new Paddles(hardwareMap);
//        flywheel = new Flywheel(hardwareMap);
//    }
//
//    private void revLauncher() {
//        flywheel.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.localizer.getPose().getPosition()));
//    }
//
//    public void revTowardGoal() {
//        revLauncher();
//        follower.setLockedHeading(getAngleToGoal());
//    }
//
//    public void fireThrough() {
//        stateTimer.resume();
//        intakeRamp.intakeThrough();
//        paddles.open();
//
//        revTowardGoal();
//
//        if (flywheel.isUpToSpeed()) {
//            intake.intake();
//        }
//    }
//
//    public void sensorLoadArtifacts() {
//        flywheel.stop();
//        intakeRamp.uptake();
//        intake.intake();
//
//        Artifact detectedArtifact = spindexer.getDetectedArtifact();
//
//        if (detectedArtifact.isArtifact()) {
//            intakedArtifact = detectedArtifact;
//            paddles.close();
//            intake.stop();
//        }
//
//        if (spindexer.artifactIsInSpindexer() && !spindexerIsRotating && intakedArtifact.isArtifact()) {
//            spindexerIsRotating = true;
//
//            boolean isFull = spindexer.intakeArtifact(intakedArtifact);
//
//            if (isFull) {
//                intake.stop();
//                intakeRamp.outtake();
//                spindexerIsRotating = false;
//                intakedArtifact = Artifact.NONE;
//                setState(State.REVVING);
//            }
//        }
//        else if (!spindexer.artifactIsInSpindexer() && spindexerIsRotating) {
//            spindexerIsRotating = false;
//            intakedArtifact = Artifact.NONE;
//            paddles.open();
//        }
//    }
//
//    public boolean isLookingAtGoal() {
//        return Math.abs(getAngleToGoal() - follower.localizer.getPose().getHeading()) < Math.toRadians(2.5);
//    }
//
//    public void firing2() {
//        if (firedArtifacts >= Robot.motifPattern.length) return;
//
//        Artifact motifArtifact = Robot.motifPattern[firedArtifacts];
//
//        if (flywheel.isAtSpeed() && (isAuto || isLookingAtGoal())) {
//            spindexer.rotateToSlot(
//                spindexer.findBestRotationToArtifact(motifArtifact)
//            );
//        }
//
//        if (spindexer.didArtifactJustDrop()) {
//            int clearedIndex = Spindexer.rollIndex(
//                (int) spindexer.currentSlotIndex
//            );
//
//            spindexer.artifacts[clearedIndex] = Artifact.NONE;
//            firedArtifacts++;
//        }
//
//        if (spindexer.getDetectedArtifact().isArtifact()) {
//            intake.setTargetPower(0.2);
//        } else {
//            intake.stop();
//        }
//
//        intakeRamp.outtake();
//
//        stateTimer.resume();
//        if (stateTimer.seconds() > 0.7) {
//            paddles.open();
//        }
//    }
//
//
//    public void revving() {
//        revTowardGoal();
//        if (!isAuto) {
//            intakeRamp.outtake();
//        }
//        intake.stop();
//
//        stateTimer.resume();
//
//        if (stateTimer.seconds() > 0.7) { // was .5
//            paddles.open();
//        }
//    }
//
//    public void continuousIntake() {
//        flywheel.stop();
//        intakeRamp.uptake();
//        intake.intake();
//        stateTimer.resume();
//
//        Artifact detectedArtifact2 = spindexer.getDetectedArtifact();
//
//        boolean artifactIsInSpindexer = spindexer.artifactIsInSpindexer();
//
//        if (detectedArtifact2.isArtifact()) {
//            if (!intakedArtifact.isArtifact()) {
//                intakedArtifact = detectedArtifact2;
//                //spindexer.intakeArtifact(intakedArtifact);
//            }
//
//            if (spindexer.getNumberOfArtifacts() == 2 && !artifactIsInSpindexer) {
//                paddles.close();
//            }
//        }
//
//        if (intakedArtifact.isArtifact() && !spindexerIsRotating && artifactIsInSpindexer
//            && stateTimer.seconds() > 0.25) {
//            spindexerIsRotating = true;
//            spindexer.intakeArtifact(intakedArtifact);
//            intakedArtifact = Artifact.NONE;
//
//            if (spindexer.getNumberOfArtifacts() == 3) {
//                spindexerIsRotating = false;
//                setState(State.REVVING);
//            }
//        }
//        else if (spindexerIsRotating && !artifactIsInSpindexer) {
//            stateTimer.resetAndStart();
//            spindexerIsRotating = false;
//        }
//    }
//
//    public void update() {
//        follower.update();
//
//        switch (state) {
//            case IDLE:
//                stateTimer.resume();
//
//                if (stateTimer.seconds() > 0.5) {
//                    paddles.open();
//                }
//
//                flywheel.stop();
//                if (intake.getTargetPower() >= 0) {
//                    intake.stop();
//                }
//                follower.setLockedHeading(null);
//                break;
//            case GROUND_FIRE:
//                fireThrough();
//                break;
//            case PADDLE_INTAKE:
//                sensorLoadArtifacts();
//                break;
//            case FIRING:
//                revTowardGoal();
//                firing2();
//                if (spindexer.getNumberOfArtifacts() == 0 && flywheel.isUpToSpeed()) { // and is up
//                    // to speed?
//                    firedArtifacts = 0;
//                    spindexer.reset();
//                    setState(State.IDLE);
//                }
//                break;
//            case AUTO_FIRE:
//                revTowardGoal();
//                firing2();
//                if (spindexer.getNumberOfArtifacts() == 0 && flywheel.isUpToSpeed()) {
//                    firedArtifacts = 0;
//                    spindexer.reset();
//                    setState(State.IDLE);
//                }
//                break;
//            case REVVING:
//                revving();
//                break;
//            case CONTINUOUS_INTAKE:
//                continuousIntake();
//                break;
//        }
//
//        double deltaTime = follower.deltaTime;
//        intake.update(deltaTime);
//
//        flywheel.update(deltaTime, follower.getVoltage());
//
//        Robot.artifacts = spindexer.artifacts;
//        Robot.currentPose = follower.localizer.getPose();
//    }
//
//    public void resetSpindexer() {
//        firedArtifacts = 0;
//        firingAllIndex = null;
//        spindexer.reset();
//        spindexer.rotateToSlot(0);
//        setState(State.IDLE);
//    }
//
//    public double getAngleToGoal() {
//        return follower.localizer.getPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
//    }
//}
