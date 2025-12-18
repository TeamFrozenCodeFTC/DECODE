package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.blackice.util.actions.Action;
import org.firstinspires.ftc.blackice.util.actions.Condition;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends TeleOps {
    boolean gamepad2Enabled = false;
    
    public void notifyFailedOperation(Condition requirement, Action action) {
        if (!requirement.isTrue()) {
            gamepad1.rumbleBlips(2);
        } else {
            action.execute();
            gamepad1.rumble(Haptics.CONFIRM);
        }
    }
    
    @Override
    public void loop() {
        int numberOfArtifacts = robot.spindexer.getNumberOfArtifacts();
        
        if (gamepad1.crossWasPressed()) {
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.GROUND_FIRE));
        } else if (gamepad1.right_trigger == 1) {
            robot.spindexerIsRotating = true;
            robot.paddlesRotatedUp = false;
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.LOAD_ARTIFACTS));
        } else if (gamepad1.rightBumperWasPressed()) {
            // robot.artifactsToFire = numberOfArtifacts;
            notifyFailedOperation(() -> numberOfArtifacts > 0,
                                  () -> robot.setState(Robot.State.SALVO));
        } else if (gamepad1.triangleWasPressed()) {
            // robot.artifactsToFire = numberOfArtifacts;
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.SENSOR_LOAD_ARTIFACTS));
        } else if (gamepad1.squareWasPressed()) { // Human Player Load
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> {
                                      robot.spindexer.rotateToSlot(0.5);
                                      robot.spindexer.dropAllIndex = 2;
                                      robot.preload(new Artifact[]
                                                        {Artifact.GREEN,
                                                            Artifact.PURPLE,
                                                            Artifact.PURPLE});
                                      robot.setState(Robot.State.IDLE);
                                  });
        } else if (gamepad1.leftBumperWasPressed()) {
            robot.setState(Robot.State.IDLE);
            gamepad1.rumble(Haptics.CONFIRM);
        } else if (gamepad1.dpadDownWasPressed()) {
            robot.flywheel.manualAdjustmentMultiplier -= 0.01;
        } else if (gamepad1.dpadUpWasPressed()) {
            robot.flywheel.manualAdjustmentMultiplier += 0.01;
        } else if (gamepad1.left_trigger == 1) {
            robot.resetSpindexer();
            robot.intake.motor.setPower(-1);
        } else if (gamepad1.dpad_right) {
            robot.follower.setCurrentHeading(
                Math.toDegrees(robot.follower.getCurrentPose().getHeading()) + 1);
        } else if (gamepad1.dpad_left) {
            robot.follower.setCurrentHeading(
                Math.toDegrees(robot.follower.getCurrentPose().getHeading()) - 1);
        }
        
        if (gamepad1.right_stick_x != 0) {
            robot.follower.lockHeadingAt(null);
        }
        if (gamepad1.optionsWasPressed()) {
            robot.follower.setCurrentPose(robot.allianceColor.getHumanPlayerZone());
            robot.follower.teleOpTarget =
                robot.follower.getMotionState().pose.headingToDegrees();
        }

        if (robot.allianceColor == AllianceColor.BLUE) {
            robot.follower.fieldCentricTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x
            );
        }
        else {
            robot.follower.fieldCentricTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
            );
        }
        
        if (gamepad1.circleWasPressed()) {
            gamepad2Enabled = !gamepad2Enabled;
            gamepad2.rumble(Haptics.CONFIRM);
            robot.setState(Robot.State.REVVING);
        }
        
        if (gamepad2.dpadUpWasPressed()) {
            robot.artifactsToFire = 0;
            robot.firedArtifacts = 0;
            robot.droppedFirstArtifact = false;
            robot.reverseSpindexerCase = false;
            robot.spindexer.resetSlots();
            robot.spindexer.rotateToSlot(0);
        }
        
        if (!gamepad2Enabled) {
        
        } else if (gamepad2.dpadRightWasPressed()) {
            robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex - 1);
        } else if (gamepad2.dpadLeftWasPressed()) {
            robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex + 1);
        } else if (gamepad2.squareWasPressed()) {
            if (!robot.spindexer._rotateToArtifact(Artifact.PURPLE)) {
                gamepad2.rumbleBlips(2);
            }
        } else if (gamepad2.circleWasPressed()) {
            if (!robot.spindexer._rotateToArtifact(Artifact.GREEN)) {
                gamepad2.rumbleBlips(2);
            }
        }
        
        super.loop();
    }
}
