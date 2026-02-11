package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.ExecutorRegistry;

import java.util.Arrays;

public class TeleOps extends OpMode {
    Robot robot;

    @Override
    public void init_loop() {
        if (gamepad1.triangleWasPressed()) {
            Robot.allianceColor = (AllianceColor.BLUE == Robot.allianceColor) ?
                AllianceColor.RED :
                AllianceColor.BLUE;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        telemetry.addData("Alliance Color (Press △)", Robot.allianceColor);
        telemetry.addData("position", Robot.currentPose);
        telemetry.addData("allianceColor", Robot.allianceColor);
        telemetry.addData("motifPattern", Arrays.deepToString(
            Robot.motifPattern.getPattern()));
        telemetry.update();
    }
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.follower.drivetrain.zeroPowerBrakeMode();
        
        if (Robot.artifacts != null) {
            robot.spindexer.artifacts = Robot.artifacts;
        }
        else {
            robot.spindexer.artifacts = new Artifact[]
                {Artifact.NONE, Artifact.NONE, Artifact.NONE};
        }
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(Robot.allianceColor.getHumanResetZone());
        
        if (Robot.currentPose != null) {
            robot.follower.setCurrentPose(Robot.currentPose);
        }
        
//        robot.spindexer.rotateToSlot(Robot.currentSpindexerIndex);
//        robot.ramp.loadToSpindexer();
//        robot.paddles.open();
    }
    
    boolean initialized = false;
    boolean moved = false;
    
    @Override
    public void loop() {
        if (moved && !initialized) {
            robot.spindexer.rotateToSlot(Robot.currentSpindexerIndex);
            robot.transfer.openPaddles();
            initialized = true;
        }

        if (gamepad1.guide) {
            if (robot.spindexer.distanceSensors.isLeftFailing()) {
                telemetry.addData("⚠ LEFT DISTANCE sensor failing",
                                  robot.spindexer.distanceSensors.getLeftDistance());
            }
            if (robot.spindexer.distanceSensors.isRightFailing()) {
                telemetry.addData("⚠ RIGHT DISTANCE sensor failing",
                                  robot.spindexer.distanceSensors.getRightDistance());
            }
            
            telemetry.addData("state", robot.state);
            telemetry.addData("leftArtifact", robot.spindexer.getLeftArtifact());
            telemetry.addData("rightArtifact", robot.spindexer.getRightArtifact());
            telemetry.addData("current rpm", "%.2f", robot.flywheel.getRPM());
            telemetry.addData("target rpm", "%.2f", robot.flywheel.getTargetRPM());
            telemetry.addData("numOfArtifacts", robot.spindexer.getNumberOfArtifacts());
            telemetry.addData("artifacts", Arrays.deepToString(robot.spindexer.artifacts));
            telemetry.addData("spindexerIndex", robot.spindexer.currentSlotIndex);
            telemetry.addData("flywheelState", robot.flywheel.getState());
            telemetry.addData("position", robot.follower.getCurrentPose());
            telemetry.addData("distanceToGoal",
                              Robot.allianceColor.getGoalPosition()
                                  .distanceTo(
                                      robot.follower.getCurrentPose().getPosition()));
            telemetry.addData("incomingArtifacts", robot.incomingArtifact);
            telemetry.addData("paddleMode", robot.transfer.getPaddleMode());
            telemetry.addData("rampMode", robot.transfer.getRampMode());
            telemetry.addData("detectedArtifact",
                              robot.spindexer.getDetectedArtifact().toString());
            telemetry.addData("isSpindexerClear", robot.spindexer.isSpindexerClear());
            telemetry.addData("isArtifactInHandoffZone", robot.spindexer.isArtifactInHandoffZone());
            telemetry.addData("leftDistance",
                              robot.spindexer.distanceSensors.getLeftDistance());
            telemetry.addData("rightDistance",
                              robot.spindexer.distanceSensors.getRightDistance());
            telemetry.addData("arePaddlesMoving", robot.transfer.arePaddlesMoving());
            telemetry.addData("isRampMoving", robot.transfer.isRampMoving());
            telemetry.addData("isLookingAtGoal", robot.isLookingAtGoal());
            telemetry.addData("stateTimer.seconds()", robot.stateTimer.seconds());
            telemetry.addData("intakeTimer", robot.intakeTimer.seconds());
            telemetry.update();
        }
        
        if (moved) {
            robot.update();
        }
    }

    @Override
    public void stop() {
        ExecutorRegistry.shutdownAll();
    }
}
