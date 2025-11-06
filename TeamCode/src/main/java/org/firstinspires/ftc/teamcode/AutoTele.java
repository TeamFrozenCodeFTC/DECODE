package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class AutoTele extends OpMode {
    Robot robot;
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.follower.drivetrain.zeroPowerBrakeMode();
        
        if (blackboard.get("allianceColor") != null) {
            robot.allianceColor = (AllianceColor) blackboard.get("allianceColor");
        } else {
            robot.allianceColor = AllianceColor.BLUE;
        }
        
        telemetry.update();
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(new Pose(72, 72, 90));
        robot.follower.teleOpTarget = new Pose(72, 72, 90);
        
        if (blackboard.get("currentPose") != null) {
            robot.follower.setCurrentPose(((Pose) blackboard.get("currentPose")));
        }
        
        robot.spindexer.rotateToSlot(2);
        robot.intakeRamp.uptake();
        robot.paddles.open();
    }
    
    boolean lockHeadingToGoal = false;
    
    /**
     * Returns the closest multiple of 90 degrees to the given heading.
     * Works for any range of input angles and normalizes result to (-180, 180].
     *
     * @param heading the current robot heading in degrees
     * @return the nearest 90°-multiple heading, normalized
     */
    public static double closestRightAngle(double heading) {
        // Normalize input to (-180, 180]
        heading = AngleUnit.normalizeDegrees(heading);
        
        // Round to nearest multiple of 90
        double rounded = Math.round(heading / 90.0) * 90.0;
        
        // Normalize result again in case of rounding to 180 or -180
        return AngleUnit.normalizeDegrees(rounded);
    }
    
    boolean isTravelingToBase;
    
    public boolean isInLaunchZone() {
        boolean isInCloseLaunchZone =
            robot.follower.getCurrentPose().getPosition().getY() > Math.abs(robot.follower.getCurrentPose().getPosition().getX()) + 72 + 8;
        boolean isInFarLaunchZone =
            -robot.follower.getCurrentPose().getPosition().getY() > Math.abs(robot.follower.getCurrentPose().getPosition().getX()) - 24 - 8;
        boolean isTooCloseToGoal =
            robot.follower.getCurrentPose().getPosition().distanceTo(robot.allianceColor.getGoalPosition()) < 63;
        return (isInCloseLaunchZone || isInFarLaunchZone) && !isTooCloseToGoal;
    }
    
    public enum State {
        GROUND_FIRE,
        UPLOAD,
        FIRING,
        IDLE,
        AUTO_TRANSITION,
    }
    
    boolean waitingForPaddles = false;
    boolean waitingForSpindexer = false;
    boolean waitingForArtifact = false;
    boolean waitForRpm = false;
    
    State state = State.IDLE;
    
    int ramp = 0;
    boolean paddlesClosed = false;
    
    ElapsedTime stateTimer = new ElapsedTime();
    
    public void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }
    
    @Override
    public void loop() {
        telemetry.addData("distanceToGoal", "%.2f", robot.distanceToGoal);
        telemetry.addData("RPM", robot.shooter.getTargetRPM());
        
        if (gamepad1.left_trigger > 0.1) {
            robot.intake.outtake();
        }
        
        if (gamepad1.leftBumperWasPressed()) {
            setState(State.IDLE);
        }
        if (gamepad1.crossWasPressed()) {
            setState(State.GROUND_FIRE);
        }
        if (gamepad1.triangleWasPressed()) {
            setState(State.UPLOAD);
        }
        if (gamepad1.squareWasPressed()) {
            setState(State.FIRING);
        }
        
        if (!isInLaunchZone() && state == State.GROUND_FIRE) {
            setState(State.UPLOAD);
        }
        
        switch (state) {
            case GROUND_FIRE:
                robot.intakeRamp.intakeThrough();
                robot.paddles.open();
                robot.revUpShooterBasedOnDistance();
                
                if (robot.shooter.isUpToSpeed()) {
                    robot.intake.intake();
                }
                
                robot.follower.teleOpTarget =
                    robot.follower.getCurrentPose().withHeading(robot.getAngleToGoal());
                lockHeadingToGoal = true;
                break;
            case UPLOAD:
                robot.intakeRamp.uptake();
                robot.intake.intake();
                
                // Wait 0.5 sec for paddles to close, then move to spindexer
                if (gamepad1.triangleWasPressed()) {
                    stateTimer.reset();
                }
                if (gamepad1.triangleWasPressed() && stateTimer.seconds() < 0.5) {
                    robot.paddles.close();
                } else if (stateTimer.seconds() < 1.0) {
                    robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex - 1);
                } else {
                    robot.paddles.open();
                    setState(State.IDLE);
                }
                break;
                
            case FIRING:
                robot.revUpShooterBasedOnDistance();
                
                if (robot.shooter.isUpToSpeed()) {
                    robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex + 1);
                }
                
                robot.follower.teleOpTarget =
                    robot.follower.getCurrentPose().withHeading(robot.getAngleToGoal());
                lockHeadingToGoal = true;
                
                // Wait 0.5s for the artifact to leave
                if (stateTimer.seconds() > 0.5) {
                    if (robot.shooter.getRpm() / robot.shooter.getTargetRPM() < 0.95) {
                        // RPM dropped too much, maybe jam or missed shot
                        // Handle this however you like
                    }
                    setState(State.IDLE);
                }
                break;
            
            case IDLE:
                robot.spindexer.partiallyRotate(robot.spindexer.currentSlotIndex);
                robot.shooter.stop();
                robot.isRevingToGoal = false;
                robot.intake.stop();
                break;
        }
        }
        
//
//        switch (state) {
//            case GROUND_FIRE:
//                robot.intakeRamp.intakeThrough();
//                robot.paddles.open();
//                robot.revUpShooterBasedOnDistance();
//                if (robot.shooter.isUpToSpeed()) {
//                    robot.intake.intake();
//                }
//                robot.follower.teleOpTarget =
//                    robot.follower.getCurrentPose().withHeading(robot.getAngleToGoal());
//                lockHeadingToGoal = true;
//                break;
//            case UPLOAD:
//                robot.intakeRamp.uptake();
//                robot.intake.intake();
//
////                Spindexer.Artifact detectedArtifact =
////                    robot.spindexer.getDetectedArtifact();
////                if (detectedArtifact.isArtifact()) {
//                Spindexer.Artifact detectedArtifact =
//                    Spindexer.Artifact.PURPLE; // Placeholder until we have a sensor
//                if (gamepad1.triangleWasPressed()) {
//                    robot.paddles.close();
//
//                    if (!waitingForPaddles) {
//                        robot.spindexer.slots[robot.spindexer.currentSlotIndex] =
//                            detectedArtifact;
//                        waitingForPaddles = true;
//                        timer.reset();
//                    }
//                }
//                break;
//            case FIRING:
//                robot.revUpShooterBasedOnDistance();
//                if (robot.shooter.isUpToSpeed()) {
//                    robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex + 1);
//                }
//                if (!waitingForArtifact) {
//                    waitingForArtifact = true;
//                    timer.reset();
//                }
//
//
//                robot.follower.teleOpTarget =
//                    robot.follower.getCurrentPose().withHeading(robot.getAngleToGoal());
//                lockHeadingToGoal = true;
//                break;
//            case IDLE:
//                robot.spindexer.partiallyRotate(robot.spindexer.currentSlotIndex);
//                robot.shooter.stop();
//                robot.isRevingToGoal = false;
//                robot.intake.stop();
//                break;
//        }
//
//        if (waitingForArtifact && timer.seconds() > 0.5) {
//            if (robot.shooter.getRpm() / robot.shooter.getTargetRPM() < 0.95) {
//                // what is a possible failure point that you see with our robot?
//                waitForRpm = true;
//            }
//        }
//
//        if (waitForRpm) {
//            if (robot.shooter.isUpToSpeed()) {
//                waitForRpm = false;
//                waitingForArtifact = false;
//            }
//            timer.reset();
//        }
//
//        if (waitingForPaddles && timer.seconds() > 0.5) {
//            robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex - 1);
//            if (robot.spindexer.currentSlotIndex == 0) {
//                state = State.IDLE;
//            }
//            waitingForPaddles = false;
//
//            if (!waitingForSpindexer) {
//                waitingForSpindexer = true;
//                timer.reset();
//            }
//        }
//        if (waitingForSpindexer && timer.seconds() > 0.5) {
//            robot.paddles.open();
//            waitingForSpindexer = false;
//        }
        
        if (gamepad1.dpad_down) {
            robot.shooter.setRPM(robot.shooter.getTargetRPM() - 50);
        }
        if (gamepad1.dpad_up) {
            robot.shooter.setRPM(robot.shooter.getTargetRPM() + 50);
        }
        
        telemetry.addData("current rpm", "%.2f", robot.shooter.getRpm());
        telemetry.addData("target rpm", "%.2f", robot.shooter.getTargetRPM());
        
        telemetry.addData("currentPose", robot.follower.getCurrentPose());
        telemetry.addData("voltage", robot.follower.getVoltage());
        
        if (gamepad1.right_stick_x != 0) {
            lockHeadingToGoal = false;
        }
        //        if (gamepad1.circleWasPressed()) {
        //            isTravelingToBase = !isTravelingToBase;
        //
        //        }
        
        if (isTravelingToBase) {
            robot.follower.holdPose(new Pose(robot.allianceColor.getBasePosition(),
                                             closestRightAngle(robot.follower.getCurrentPose().getHeading())));
            robot.follower.teleOpTarget =
                robot.follower.getMotionState().pose;
        }
        else if (lockHeadingToGoal) {
            telemetry.addData("locked heading to", robot.getAngleToGoal());
            robot.follower.goalCentricTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                robot.getAngleToGoal()
            );
        }
        else {
            telemetry.addLine("driving");
            robot.follower.fieldCentricTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x
            );
        }
        
        telemetry.update();
        
        robot.update();
    }
}
