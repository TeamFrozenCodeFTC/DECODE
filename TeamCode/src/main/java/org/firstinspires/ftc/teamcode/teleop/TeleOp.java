package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.function.BooleanSupplier;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends TeleOps {
    boolean gamepad2Enabled = false;
    
    public void notifyFailedOperation(BooleanSupplier requirement, Runnable action) {
        if (!requirement.getAsBoolean()) {
            gamepad1.rumbleBlips(2);
        } else {
            action.run();
            gamepad1.rumble(Haptics.CONFIRM);
        }
    }
    
    boolean leftTriggerWasPressed = false;
    
    @Override
    public void loop() {
        if (gamepad1.options) {
            return;
        }

        int numberOfArtifacts = robot.spindexer.getNumberOfArtifacts();
        
        if (gamepad1.crossWasPressed()) {
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.FIRE_THROUGH));
        } else if (gamepad1.right_trigger == 1) {
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.INTAKING));
        } else if (gamepad1.rightBumperWasPressed()) {
            notifyFailedOperation(() -> numberOfArtifacts > 0,
                                  () -> robot.setState(Robot.State.LAUNCHING));
        } else if (gamepad1.squareWasPressed()) { // Human Player Load
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  this::humanPlayerLoad);
        } else if (gamepad1.leftBumperWasPressed()) {
            robot.setState(Robot.State.IDLE);
            //robot.spindexer.servo.setPwmDisable();
            robot.transfer.openPaddles();
            robot.transfer.update();
            gamepad1.rumble(Haptics.CONFIRM);
        } else if (gamepad1.dpadDownWasPressed()) {
            robot.flywheel.manualAdjustmentMultiplier -= 0.01;
        } else if (gamepad1.dpadUpWasPressed()) {
            robot.flywheel.manualAdjustmentMultiplier += 0.01;
        } else if (gamepad1.left_trigger == 1 && !leftTriggerWasPressed) {
            leftTriggerWasPressed = true;
            robot.resetSpindexer();
            robot.intake.outtake();
            robot.incomingArtifact = Artifact.NONE;
            robot.intake.motor.setPower(-1);
            
            // NEW
            robot.transfer.openPaddles();
            robot.transfer.update();
        }
        
        if (gamepad1.left_trigger == 0 && leftTriggerWasPressed) {
            leftTriggerWasPressed = false;
            robot.intake.stop();
        }
        
        if (gamepad1.right_stick_x != 0) {
            robot.follower.setLockedHeading(null);
        }
        if (gamepad1.optionsWasPressed()) {
            robot.follower.setCurrentPose(Robot.allianceColor.getHumanResetZone());
        }
        if (gamepad1.shareWasPressed()) {
            robot.follower.setCurrentPose(Robot.allianceColor.getGoalReset());
        }

        if (Robot.allianceColor == AllianceColor.BLUE) {
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
        
        if (gamepad1.left_stick_y > 0 || gamepad1.left_stick_x > 0) {
            moved = true;
        }
        
        if (gamepad1.circleWasPressed()) {
            gamepad2Enabled = true;
            gamepad2.rumble(Haptics.CONFIRM);
            robot.setState(Robot.State.REVVING);
            robot.transfer.feedFromSpindexer();
        }
        
        if (gamepad1.dpadRightWasPressed()) {
            robot.spindexer.rotateRight();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            robot.spindexer.rotateLeft();
        }
        
        
        super.loop();
    }
    
    public void humanPlayerLoad() {
        robot.spindexer.rotateToSlot(0.5);
        robot.preload(new Artifact[]
                          {Artifact.GREEN,
                              Artifact.PURPLE,
                              Artifact.PURPLE});
        robot.setState(Robot.State.IDLE);
    }
}
