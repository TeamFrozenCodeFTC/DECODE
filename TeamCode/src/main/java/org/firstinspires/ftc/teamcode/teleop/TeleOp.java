package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.blackice.util.actions.Action;
import org.firstinspires.ftc.blackice.util.actions.Condition;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Arrays;
import java.util.Collections;

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
    
    private static final int QUEUE_SIZE = 3;
    private Artifact[] firingQueue = new Artifact[QUEUE_SIZE];
    private int queueCount = 0;
    
    private void enqueueArtifact(Artifact artifact) {
        int numberOfArtifactsLeft = robot.spindexer.count(artifact) - Collections.frequency(
            Arrays.asList(firingQueue), artifact);
        if (queueCount >= QUEUE_SIZE || numberOfArtifactsLeft <= 0) {
            return;
        }
        
        firingQueue[queueCount] = artifact;
        queueCount++;
    }
    
    @Override
    public void loop() {
        if (gamepad1.options) {
            return;
        }
        
        if (robot.spindexer.getNumberOfArtifacts() == 0
            && robot.intakedArtifact == Artifact.NONE
            && robot.spindexer.artifactIsInSpindexer()
            && !robot.spindexer.getDetectedArtifact().isArtifact()) {
            robot.paddles.close();
            robot.spindexer.rotateToSlot(0.5);
            robot.preload(new Artifact[]
                              {Artifact.GREEN,
                                  Artifact.PURPLE,
                                  Artifact.PURPLE});
            robot.setState(Robot.State.IDLE);
        }
        
        int numberOfArtifacts = robot.spindexer.getNumberOfArtifacts();
        
        if (gamepad1.crossWasPressed()) {
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.GROUND_FIRE));
        } else if (gamepad1.right_trigger == 1) {
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.CONTINUOUS_INTAKE));
        } else if (gamepad1.rightBumperWasPressed()) {
            notifyFailedOperation(() -> numberOfArtifacts > 0,
                                  () -> robot.setState(Robot.State.FIRING));
        } else if (gamepad1.rightStickButtonWasPressed()) {
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.STUFF_INTAKE));
//        } else if (gamepad1.circleWasPressed()) {
//            notifyFailedOperation(() -> numberOfArtifacts > 0,
//                                  () -> robot.setState(Robot.State.MOTIF_FIRING));
        } else if (gamepad1.triangleWasPressed()) {
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  () -> robot.setState(Robot.State.PADDLE_INTAKE));
        } else if (gamepad1.squareWasPressed()) { // Human Player Load
            notifyFailedOperation(() -> numberOfArtifacts < 3,
                                  this::humanPlayerLoad);
        } else if (gamepad1.leftBumperWasPressed()) {
            robot.setState(Robot.State.IDLE);
            gamepad1.rumble(Haptics.CONFIRM);
        } else if (gamepad1.dpadDownWasPressed()) {
            robot.flywheel.manualAdjustmentMultiplier -= 0.01;
        } else if (gamepad1.dpadUpWasPressed()) {
            robot.flywheel.manualAdjustmentMultiplier += 0.01;
        } else if (gamepad1.left_trigger == 1) {
            robot.resetSpindexer();
            robot.intake.outtake(); // new
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
            robot.follower.setCurrentPose(Robot.allianceColor.getHumanResetZone());
            robot.follower.teleOpTarget =
                robot.follower.getMotionState().pose.headingToDegrees();
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
        
        if (gamepad1.circleWasPressed()) {
            gamepad2Enabled = true;
            gamepad2.rumble(Haptics.CONFIRM);
            robot.setState(Robot.State.REVVING);
        }
        
        if (gamepad2.dpadUpWasPressed()) {
            robot.firedArtifacts = 0;
            robot.spindexer.resetSlots();
            robot.spindexer.rotateToSlot(0);
        }
        
        if (!gamepad2Enabled) {
        
        } else if (gamepad2.dpadRightWasPressed()) {
            robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex - 1);
        } else if (gamepad2.dpadLeftWasPressed()) {
            robot.spindexer.rotateToSlot(robot.spindexer.currentSlotIndex + 1);
        } else if (gamepad2.squareWasPressed()) {
            enqueueArtifact(Artifact.PURPLE);
        }
        else if (gamepad2.circleWasPressed()) {
            enqueueArtifact(Artifact.GREEN);
        }

        if (gamepad2Enabled && robot.flywheel.isUpToSpeed()) {
            if (robot.firedArtifacts == 3 || robot.spindexer.getNumberOfArtifacts() == 0) {
                firingQueue = new Artifact[QUEUE_SIZE];
                queueCount = 0;
                gamepad2Enabled = false;
            }
            else {
                robot.spindexer.rotateToArtifact(firingQueue[robot.firedArtifacts]);
            } // slight optimization of rotating twice sometimes
        }
        
        
        if (robot.state == Robot.State.IDLE) {
            gamepad2Enabled = false;
        }
        
        // TODO
        // create a queue so silas can spam the order and it remembers
        // test mini black ice
        // test motif firing for auto
        // create auto
        // black ice curves

        
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
