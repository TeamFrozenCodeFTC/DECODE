package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    Robot robot;
    
    public static Pose startingPose = new Pose(72, 72, 90);
    
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
        
        double currentVoltage = robot.follower.drivePowerController.getVoltage();
        telemetry.addData("Initializing Voltage", "%.2f V", currentVoltage);
        telemetry.update();
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(startingPose);

        if (blackboard.get("currentPose") != null) {
            robot.follower.setCurrentPose(((Pose) blackboard.get("currentPose")));
        }
        
        robot.follower.teleOpTarget = robot.follower.getCurrentPose().headingToDegrees();
        
        robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex);
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
  
    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.1) {
            robot.intake.outtake();
        }
        
        if (gamepad1.leftBumperWasPressed()) {
            robot.setState(Robot.State.IDLE);
            gamepad1.rumble(Haptics.CONFIRM);
        }
        if (gamepad1.crossWasPressed()) {
            robot.setState(Robot.State.GROUND_FIRE);
            gamepad1.rumble(Haptics.CONFIRM);
        }
        if (gamepad1.triangleWasPressed()) {
            robot.setState(Robot.State.LOAD_ARTIFACTS);
            gamepad1.rumble(Haptics.CONFIRM);
        }
        if (gamepad1.squareWasPressed()) {
            robot.setState(Robot.State.FIRING);
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        if (gamepad1.dpad_down) {
            robot.shooter.setRPM(robot.shooter.getTargetRPM() - 50);
        }
        if (gamepad1.dpad_up) {
            robot.shooter.setRPM(robot.shooter.getTargetRPM() + 50);
        }

        if (gamepad1.circleWasPressed()) {
            robot.detectedArtifact = Spindexer.Artifact.PURPLE;
            gamepad1.rumble(Haptics.CONFIRM);
        }

//        telemetry.addData("isInLaunchZone", isInLaunchZone());
        telemetry.addData("state", robot.state.toString());
        telemetry.addData("currentSlot", robot.spindexer.currentSlotIndex);
//        telemetry.addData("detectedArtifact",
//                          robot.spindexer.getDetectedArtifact().toString());
//
//        telemetry.addData("right hue",
//                          robot.spindexer.rightColorSensor.hue);
//        telemetry.addData("right sat",
//                          robot.spindexer.rightColorSensor.saturation);
//        telemetry.addData("right distance cm",
//                          robot.spindexer.rightColorSensor.distanceCm);
//
//        telemetry.addData("left hue",
//                          robot.spindexer.leftColorSensor.hue);
//        telemetry.addData("left sat",
//                          robot.spindexer.leftColorSensor.saturation);
//        telemetry.addData("left distance cm",
//                          robot.spindexer.leftColorSensor.distanceCm);

        telemetry.addData("current rpm", "%.2f", robot.shooter.getRpm());
        telemetry.addData("target rpm", "%.2f", robot.shooter.getTargetRPM());

        telemetry.addData("Hz", 1/robot.follower.getMotionState().deltaTime);
        
        if (gamepad1.right_stick_x != 0) {
            robot.follower.lockHeadingAt(null);
        }
        
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
