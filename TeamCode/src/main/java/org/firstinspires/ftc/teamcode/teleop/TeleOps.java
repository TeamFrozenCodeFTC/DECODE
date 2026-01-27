package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.spindexer.Spindexer;

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
    
    @Override
    public void loop() {
        if (robot.spindexer.distanceSensors.leftDistanceSensorIsFailing()) {
            telemetry.addData("⚠ RIGHT DISTANCE sensor failing",
                              robot.spindexer.distanceSensors.getLeftDistance());
            telemetry.update();
        }
        if (robot.spindexer.distanceSensors.rightDistanceSensorIsFailing()) {
            telemetry.addData("⚠ RIGHT DISTANCE sensor failing",
                              robot.spindexer.distanceSensors.getRightDistance());
            telemetry.update();
        }

        if (gamepad1.guide) {
            int leftIndex = robot.spindexer.shiftLeft(1);
            int rightIndex = robot.spindexer.shiftRight(1);
            
            int leftSlotIndex = Spindexer.rollIndex(leftIndex);
            int rightSlotIndex = Spindexer.rollIndex(rightIndex);
            
            telemetry.addData("leftIndex", leftIndex);
            telemetry.addData("rightIndex", rightIndex);
            telemetry.addData("leftSlotIndex", leftSlotIndex);
            telemetry.addData("rightSlotIndex", rightSlotIndex);
            telemetry.addData("leftIsArtifact", robot.spindexer.artifacts[leftSlotIndex]);
            telemetry.addData("rightIsArtifact", robot.spindexer.artifacts[rightSlotIndex]);
            telemetry.addData("state", robot.state);
            telemetry.addData("current rpm", "%.2f", robot.flywheel.getRPM());
            telemetry.addData("target rpm", "%.2f", robot.flywheel.getTargetRPM());
            telemetry.addData("numOfArtifacts", robot.spindexer.getNumberOfArtifacts());
            telemetry.addData("artifacts", Arrays.deepToString(robot.spindexer.artifacts));
            telemetry.addData("spindexer index", robot.spindexer.currentSlotIndex);
            telemetry.addData("isUpToSpeed", robot.flywheel.isAtSpeed());
            telemetry.addData("position", robot.follower.getCurrentPose());
            telemetry.addData("distanceToGoal",
                              Robot.allianceColor.getGoalPosition()
                                  .distanceTo(
                                      robot.follower.getCurrentPose().getPosition()));
            telemetry.addData("incomingArtifacts", robot.incomingArtifact);
            telemetry.update();
        }
        
        robot.update();
    }
}
