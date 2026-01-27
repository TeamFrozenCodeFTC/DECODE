//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.Artifact;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.spindexer.Spindexer;
//import org.firstinspires.ftc.teamcode.utils.Timeout;
//
//public class ArtifactFlow {
//    public Spindexer spindexer;
//    public Paddles paddles;
//    public Ramp ramp;
//    public Intake intake;
//    public Flywheel flywheel;
//
//    public Artifact incomingArtifact = Artifact.NONE;
//
//    public int firedArtifacts = 0;
//
//    public enum FlowState {
//        IDLE,
//        INTAKING,
//        REVVING,
//        LAUNCHING
//    }
//
//    FlowState state = FlowState.IDLE;
//
//    public ArtifactFlow(HardwareMap hardwareMap) {
//        this.spindexer = new Spindexer(hardwareMap);
//        this.paddles = new Paddles(hardwareMap);
//        this.ramp = new Ramp(hardwareMap);
//        this.intake = new Intake(hardwareMap);
//        this.flywheel = new Flywheel(hardwareMap);
//    }
//
//    public void update() {
//        flywheel.readSensors();
//        paddles.update();
//        ramp.update();
//
//        switch (state) {
//            case IDLE:
//                intake.stop();
//                if (ramp.isStationary()) {
//                    paddles.open();
//                }
//                if (paddles.isStationary()) {
//                    ramp.loadToSpindexer();
//                }
//                flywheel.stop();
//                firedArtifacts = 0;
//                break;
//            case REVVING:
//                flywheel.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.localizer.getPose().getPosition();
//                follower.setLockedHeading(getAngleToGoal());
//                break;
//            case INTAKING:
//                intake();
//                if (spindexer.getNumberOfArtifacts() == 3) {
//                    state = FlowState.REVVING;
//                    return;
//                }
//                break;
//            case LAUNCHING:
//                launch();
//                if (flywheel.isAtSpeed() && spindexer.getNumberOfArtifacts() == 0) {
//                    firedArtifacts = 0;
//                    spindexer.rotateToSlot(0);
//                    state = FlowState.IDLE;
//                    return;
//                }
//                break;
//        }
//
//        if (incomingArtifact.isArtifact()) {
//            if (ramp.isStationary()) {
//                paddles.close();
//            }
//            if (spindexer.didArtifactJustEnterSpindexer()) {
//                spindexer.intakeArtifact(incomingArtifact, motifPattern);
//                incomingArtifact = Artifact.NONE;
//            }
//        }
//
//        intake.update(dt);
//        flywheel.update(dt, voltage);
//    }
////
////    private void revLauncher() {
////        flywheel.setRpmFromDistance(allianceColor.getGoalPosition().distanceTo(follower.localizer.getPose().getPosition()));
////    }
////
////    public void revTowardGoal() {
////        revLauncher();
////        follower.setLockedHeading(getAngleToGoal());
////    }
//
////    public double getAngleToGoal() {
////        return follower.localizer.getPose().getPosition().getAngleToLookAt(allianceColor.getGoalPosition());
////    }
//
//    Timeout intakeTimer = new Timeout();
//
//    public void intake() {
//        Artifact detectedArtifact = spindexer.getDetectedArtifact();
//        int count = spindexer.getNumberOfArtifacts();
//
//        if (detectedArtifact.isArtifact() && !incomingArtifact.isArtifact()) {
//            incomingArtifact = detectedArtifact;
//            spindexer.artifacts[Spindexer.rollIndex((int) spindexer.currentSlotIndex)] =
//                incomingArtifact;
//            intakeTimer.resetAndStart();
//        }
//
//        if (incomingArtifact.isArtifact()
//            && spindexer.didArtifactJustEnterSpindexer()
//            && intakeTimer.seconds() > 0.25) {
//
//            spindexer.intakeArtifact(incomingArtifact, motifPattern);
//            incomingArtifact = Artifact.NONE;
//        }
//    }
//
//    public void launch() {
//        Artifact motifArtifact = Robot.motifPattern[firedArtifacts];
//
//        if (spindexer.didArtifactJustDrop()) {
//            spindexer.artifacts[Spindexer.rollIndex((int) spindexer.currentSlotIndex)] = Artifact.NONE;
//        }
//
//        if (flywheel.isAtSpeed()) {
//            spindexer.rotateToSlot(spindexer.findBestRotationToArtifact(motifArtifact));
//        }
//
//        if (paddles.isStationary()) {
//            ramp.feedFromSpindexer();
//        }
//        if (ramp.isStationary()) {
//            paddles.open();
//        }
//    }
//}
