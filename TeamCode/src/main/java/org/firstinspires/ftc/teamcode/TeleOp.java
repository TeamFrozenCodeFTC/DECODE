package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
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
        
        ElapsedTime initTimer = new ElapsedTime();
        while (initTimer.seconds() < .5) {

        }
    }
    
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(new Pose(72, 72, 90));
        robot.follower.teleOpTarget = new Pose(72, 72, 90);
        
        if (blackboard.get("currentPose") != null) {
            robot.follower.setCurrentPose(((Pose) blackboard.get("currentPose")));
        }
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
    
    
    int ramp = 0;
    boolean paddlesClosed = false;
    
    @Override
    public void loop() {
        telemetry.addData("distanceToGoal", "%.2f", robot.distanceToGoal);
        telemetry.addData("RPM", robot.shooter.getTargetRPM());
        
        if (robot.shooter.isUpToSpeed() && isInLaunchZone()) {
            robot.intake.intake();
        }
        
        // make shooter unrev once artifact slowed it down
        
        if (gamepad1.rightBumperWasPressed()) {
            robot.revUpShooterBasedOnDistance();
        }
        if (gamepad1.leftBumperWasPressed()) {
            robot.shooter.stop();
            robot.isRevingToGoal = false;
            robot.intake.stop();
        }
        
        if (gamepad1.triangleWasPressed()) {
            ramp = 0;
        }
        if (gamepad1.touchpadWasPressed()) {
            ramp = 1;
        }
        if (gamepad1.circleWasPressed()) {
            ramp = 2;
        }
        
        if (ramp == 0) {
            robot.intakeRamp.intakeThrough();
        }
        else if (ramp == 1) {
            robot.intakeRamp.uptake();
        }
        else if (ramp == 2) {
            robot.intakeRamp.outtake();
        }
        
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.intake();
        }
        if (gamepad1.left_trigger > 0.1) {
            robot.intake.outtake();
        }
        if (gamepad1.cross) {
            robot.intake.stop();
            robot.shooter.setRPM(-500);
            robot.follower.teleOpTarget =
                robot.follower.getCurrentPose().withHeading(robot.getAngleToGoal());
            lockHeadingToGoal = true;
        }
        
        if (paddlesClosed) {
            robot.paddles.close();
        }
        else {
            
            robot.paddles.open();
        }
        
        if (gamepad1.squareWasPressed()) {
             paddlesClosed = !paddlesClosed;
        }
        
        if (gamepad1.dpadLeftWasPressed()) {
            robot.spindexer.rotateToSlot(
                Spindexer.rollIndex(robot.spindexer.currentSlotIndex + 1));
        }
        if (gamepad1.dpadRightWasPressed()) {
            robot.spindexer.partiallyRotate(
                Spindexer.rollIndex(robot.spindexer.currentSlotIndex - 1));
        }
        
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
            robot.follower.lockHeadingAt(robot.getAngleToGoal());
        }
        else {
            robot.follower.lockHeadingAt(null);
            telemetry.addLine("driving");
        }
        
        robot.follower.fieldCentricTeleOpDrive(
            gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            -gamepad1.right_stick_x
        );
        
        telemetry.update();
        
        robot.update();
    }
}
