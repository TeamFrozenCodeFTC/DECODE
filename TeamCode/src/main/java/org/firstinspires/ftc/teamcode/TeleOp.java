package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.util.actions.Action;
import org.firstinspires.ftc.blackice.util.actions.Condition;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.ArrayList;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    Robot robot;
    
    public static Pose startingPose = new Pose(8+24*5, 8.5, 0);
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());

        robot.follower.drivetrain.zeroPowerFloatMode();
        // robot.follower.drivetrain.zeroPowerBrakeMode();
        
        if (blackboard.get("allianceColor") != null) {
            robot.allianceColor = (AllianceColor) blackboard.get("allianceColor");
        }
        if (blackboard.get("motifPattern") != null) {
            robot.motifPattern = (Artifact[]) blackboard.get("motifPattern");
        }
  
        
        double currentVoltage = robot.follower.drivePowerController.getVoltage();
        telemetry.addData("Initializing Voltage", "%.2f V", currentVoltage);
        telemetry.addData("allianceColor", robot.allianceColor);
        telemetry.addData("motifPattern", Arrays.deepToString(robot.motifPattern));
        telemetry.update();
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(startingPose);

        if (blackboard.get("currentPose") != null) {
            robot.follower.setCurrentPose(((Pose) blackboard.get("currentPose")));
        }
        
        robot.follower.teleOpTarget = robot.follower.getCurrentPose().headingToDegrees();
        
        robot.spindexer.rotateToSlot(0);
        robot.spindexer.slots = new Artifact[]
            {Artifact.NONE, Artifact.NONE, Artifact.NONE};;
        robot.intakeRamp.uptake();
        robot.paddles.open();
    }
    
    boolean isTravelingToBase = false;
    
    public boolean isInLaunchZone() {
        Pose pose = robot.follower.getCurrentPose();
        double x = pose.getPosition().getX();
        double y = pose.getPosition().getY();
        
        // Center of field
        final double FIELD_CENTER_X = 72;
        
        // Top (close) launch zone: triangle with slope ±1, apex at (72, 144), 72 in deep
        boolean isInCloseLaunchZone =
            y > -Math.abs(x - FIELD_CENTER_X) + 72;  // opens downward from top wall (y=144)
        
        // Bottom (far) launch zone: inverted triangle with slope ±1, apex at (72, 0), 24 in tall
        boolean isInFarLaunchZone =
            y < Math.abs(x - FIELD_CENTER_X) + 24;   // opens upward from bottom wall (y=0)
        
        // Too close to goal check (keep your original)
        boolean isTooCloseToGoal =
            pose.getPosition().distanceTo(robot.allianceColor.getGoalPosition()) < 63;
        
        // Final condition
        return (isInCloseLaunchZone || isInFarLaunchZone) && !isTooCloseToGoal;
    }
    
    public void notifyFailedOperation(Condition requirement, Action action) {
        if (!requirement.isTrue()) {
            gamepad1.rumbleBlips(2);
        }
        else {
            action.execute();
            gamepad1.rumble(Haptics.CONFIRM);
        }
    }
    
    boolean gamepad2Enabled = false;

    @Override
    public void loop() {
        int numberOfArtifacts = robot.spindexer.getNumberOfArtifacts();
        
        if (gamepad1.options) {
        
        }
        else if (gamepad1.crossWasPressed()) {
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.GROUND_FIRE));
        }
        else if (gamepad1.triangleWasPressed()) {
           // robot.paddlesRotated = false;
            if (robot.state == Robot.State.REVVING) {
                robot.spindexer.rotateToSlot(0);
            }
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.LOAD_ARTIFACTS));
        }
        else if (gamepad1.right_trigger == 1) {
            robot.artifactsToFire = numberOfArtifacts;
            notifyFailedOperation(() -> numberOfArtifacts > 0,
                                  () -> robot.setState(Robot.State.FAST_FIRING));
        }
        else if (gamepad1.rightBumperWasPressed()) {
            robot.artifactsToFire = numberOfArtifacts;
            notifyFailedOperation(() -> numberOfArtifacts > 0,
                                  () -> robot.setState(Robot.State.FIRING));
        }
        else if (gamepad1.squareWasPressed()) { // Human Player Load
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> {
                                      robot.spindexer.rotateToSlot(0.5);
                                      robot.spindexer.slots = new Artifact[]
                                             {Artifact.GREEN,
                                              Artifact.PURPLE,
                                              Artifact.PURPLE};
                                      robot.setState(Robot.State.IDLE);
                                  });
        }
        else if (gamepad1.leftBumperWasPressed()) {
            robot.setState(Robot.State.IDLE);
            gamepad2Enabled = false;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        else if (gamepad1.dpad_down) {
            robot.launcher.setRPM(robot.launcher.getTargetRPM() - 50);
        }
        else if (gamepad1.dpad_up) {
            robot.launcher.setRPM(robot.launcher.getTargetRPM() + 50);
        }
        else if (gamepad1.dpad_right) {
            robot.follower.setCurrentHeading(Math.toDegrees(robot.follower.getCurrentPose().getHeading()) + 1);
        }
        else if (gamepad1.dpad_left) {
            robot.follower.setCurrentHeading(Math.toDegrees(robot.follower.getCurrentPose().getHeading()) - 1);
        }
        
        if (gamepad1.right_stick_x != 0) {
            robot.follower.lockHeadingAt(null);
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            robot.follower.setCurrentPose(startingPose);
        }
        
        // Auto BASE travel
//        if (gamepad1.touchpadWasPressed()) {
//            isTravelingToBase = !isTravelingToBase;
//        }
        
//        if (isTravelingToBase) {
//            robot.follower.setCurrentPose(
//                robot.follower.getCurrentPose()
//                      .addedX(-gamepad1.left_stick_y * 0.01)
//                      .addedY(-gamepad1.right_stick_x * 0.01)
//                      .addedHeading(gamepad1.right_stick_x * 0.01));
//
//            robot.follower.holdPose(new Pose(robot.allianceColor.getBasePosition(),
//                                             closestRightAngle(robot.follower.getCurrentPose().getHeading())));
//            robot.follower.teleOpTarget =
//                robot.follower.getMotionState().pose; // to degrees tho
//        }
        
        robot.follower.fieldCentricTeleOpDrive(
            gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            -gamepad1.right_stick_x
        );


        robot.update(telemetry);

        if (gamepad1.left_trigger > 0.1) {
            robot.intake.outtake();
        }
        
        if (gamepad1.circleWasPressed()) {
            gamepad2Enabled = true;
            gamepad2.rumble(Haptics.CONFIRM);
            robot.setState(Robot.State.REVVING);
        }
        
        if (gamepad2.dpadUpWasPressed()) {
            robot.spindexer.rotateToSlot(0);
            robot.spindexer.resetSlots();
        }
//        else if (gamepad2Enabled && robot.launcher.isUpToSpeed()) {
//            if (gamepad2.dpadLeftWasPressed()) {
//                robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex - 1);
//            }
//            else if (gamepad2.dpadRightWasPressed()) {
//                robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex + 1);
//            }
//            else if (gamepad2.squareWasPressed()) {
//                robot.spindexer.rotateToArtifact(Artifact.PURPLE);
//            }
//            else if (gamepad2.circleWasPressed()) {
//                robot.spindexer.rotateToArtifact(Artifact.GREEN);
//            }
//        }
//
        
        if (!gamepad2Enabled) {
        
        }
        else if (gamepad2.dpadLeftWasPressed()) {
            robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex - 1);
        }
        else if (gamepad2.dpadRightWasPressed()) {
            robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex + 1);
        }
        else if (gamepad2.squareWasPressed()) {
            if (!robot.spindexer.rotateToArtifact(Artifact.PURPLE)) {
                gamepad2.rumbleBlips(2);
            }
        }
        else if (gamepad2.circleWasPressed()) {
            if (!robot.spindexer.rotateToArtifact(Artifact.GREEN)) {
                gamepad2.rumbleBlips(2);
            }
        }
    }
    
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
}
