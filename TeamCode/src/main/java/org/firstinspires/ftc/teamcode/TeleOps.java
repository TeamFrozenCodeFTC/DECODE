package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.util.Timeout;
import org.firstinspires.ftc.blackice.util.actions.Action;
import org.firstinspires.ftc.blackice.util.actions.Condition;
import org.firstinspires.ftc.blackice.util.geometry.Pose;

import java.util.Arrays;

public class TeleOps extends OpMode {
    Robot robot;

    @Override
    public void init_loop() {
        if (gamepad1.triangleWasPressed()) {
            robot.allianceColor = (AllianceColor.BLUE == robot.allianceColor) ?
                AllianceColor.RED :
                AllianceColor.BLUE;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        telemetry.addData("Alliance Color (Press △)", robot.allianceColor);
        telemetry.addData("position", robot.follower.getCurrentPose());
        telemetry.update();
    }
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.follower.drivetrain.zeroPowerBrakeMode();
        
        if (blackboard.get("allianceColor") != null) {
            robot.allianceColor = (AllianceColor) blackboard.get("allianceColor");
        }
        if (blackboard.get("motifPattern") != null) {
            robot.motifPattern = (Artifact[]) blackboard.get("motifPattern");
        }
        if (blackboard.get("slots") != null) {
            robot.spindexer.slots = (Artifact[]) blackboard.get("slots");
        }
        else {
            robot.spindexer.slots = new Artifact[]
                {Artifact.NONE, Artifact.NONE, Artifact.NONE};
        }
        
        double currentVoltage = robot.follower.drivePowerController.getVoltage();
        telemetry.addData("Initializing Voltage", "%.2f V", currentVoltage);
        telemetry.addData("allianceColor", robot.allianceColor);
        telemetry.addData("motifPattern", Arrays.deepToString(robot.motifPattern));
        telemetry.update();
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(robot.allianceColor.getHumanPlayerZone());
        
        if (blackboard.get("currentPose") != null) {
            robot.follower.setCurrentPose(((Pose) blackboard.get("currentPose")));
        }
        
        robot.follower.teleOpTarget = robot.follower.getCurrentPose().headingToDegrees();
        
        robot.spindexer.rotateToSlot(0);
        robot.intakeRamp.uptake();
        robot.paddles.open();
    }

    int numberOfArtifacts = 0;

    @Override
    public void loop() {
        if (gamepad1.guide) {
            telemetry.addData("state", robot.state);
            telemetry.addData("current rpm", "%.2f", robot.launcher.getRpm());
            telemetry.addData("target rpm", "%.2f", robot.launcher.getTargetRPM());
            telemetry.addData("numOfArtifacts", numberOfArtifacts);
            telemetry.addData("artifacts", Arrays.deepToString(robot.spindexer.slots));
            telemetry.addData("spindexer index", robot.spindexer.currentSlotIndex);
            telemetry.addData("spindexerHasRotated", robot.spindexerHasRotated);
            telemetry.addData("paddlesRotated", robot.paddlesRotatedUp);
            telemetry.addData("isUpToSpeed", robot.launcher.isUpToSpeed());
            telemetry.addData("position", robot.follower.getCurrentPose());
            telemetry.addData("distanceToGoal",
                              robot.allianceColor.getGoalPosition()
                                  .distanceTo(
                                      robot.follower.getCurrentPose().getPosition()));
            telemetry.update();
        }
        
        robot.intake.update();
        
        robot.launcher.update(robot.follower.getMotionState().deltaTime,
                              robot.follower.getVoltage());
    }
}
