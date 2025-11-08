package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class AutoTele extends OpMode {
    Robot2 robot;
    
    @Override
    public void init() {
        robot = new Robot2(hardwareMap);
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
        Pose startingPose = new Pose(72, 72, 90);
        if (robot.allianceColor == AllianceColor.RED) {
            startingPose = startingPose.mirroredAcrossYAxis();
        }
        robot.follower.setCurrentPose(startingPose);
        robot.follower.teleOpTarget = startingPose;
        
        if (blackboard.get("currentPose") != null) {
            robot.follower.setCurrentPose(((Pose) blackboard.get("currentPose")));
        }
        
        robot.spindexer.rotateToSlot(0);
        robot.intakeRamp.uptake();
        robot.paddles.open();
    }
    
    boolean lockHeadingToGoal = false;
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
  
    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.1) {
            robot.intake.outtake();
        }
        
        if (gamepad1.leftBumperWasPressed()) {
            robot.setState(Robot2.State.IDLE);
        }
        if (gamepad1.crossWasPressed()) {
            robot.setState(Robot2.State.GROUND_FIRE);
        }
        if (gamepad1.triangleWasPressed()) {
            robot.setState(Robot2.State.LOAD_ARTIFACTS);
        }
        if (gamepad1.squareWasPressed()) {
            robot.setState(Robot2.State.FIRING);
        }
        
        robot.detectedArtifact = gamepad1.circleWasPressed();
        
        telemetry.addData("state", robot.state.toString());
        telemetry.addData("currentSlot", robot.spindexer.currentSlotIndex);
        telemetry.addData("detectedArtifact",
                          robot.spindexer.getDetectedArtifact(telemetry).toString());
        telemetry.addData("distanceToGoal", "%.2f", robot.distanceToGoal);

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
            
            robot.follower.lockHeadingAt(null);
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
        
        robot.follower.fieldCentricTeleOpDrive(
            gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            -gamepad1.right_stick_x
        );
        
        telemetry.update();
        
        robot.update();
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
