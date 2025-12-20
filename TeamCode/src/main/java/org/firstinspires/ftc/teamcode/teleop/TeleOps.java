package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;

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
        telemetry.addData("motifPattern", Arrays.deepToString(Robot.motifPattern));
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
        
        robot.follower.teleOpTarget = robot.follower.getCurrentPose().headingToDegrees();
        
        robot.spindexer.rotateToSlot(0);
        robot.intakeRamp.uptake();
        robot.paddles.open();
    }
    
    @Override
    public void loop() {
        if (gamepad1.guide) {
            telemetry.addData("state", robot.state);
            telemetry.addData("current rpm", "%.2f", robot.flywheel.getRpm());
            telemetry.addData("target rpm", "%.2f", robot.flywheel.getTargetRPM());
            telemetry.addData("numOfArtifacts", robot.spindexer.getNumberOfArtifacts());
            telemetry.addData("artifacts", Arrays.deepToString(robot.spindexer.artifacts));
            telemetry.addData("spindexer index", robot.spindexer.currentSlotIndex);
            telemetry.addData("spindexerIsRotating", robot.spindexerIsRotating);
            telemetry.addData("paddlesRotated", robot.paddlesRotatedUp);
            telemetry.addData("isUpToSpeed", robot.flywheel.isUpToSpeed());
            telemetry.addData("position", robot.follower.getCurrentPose());
            telemetry.addData("distanceToGoal",
                              Robot.allianceColor.getGoalPosition()
                                  .distanceTo(
                                      robot.follower.getCurrentPose().getPosition()));
            telemetry.addData("firingAllIndex", robot.firingAllIndex);
            telemetry.update();
        }
        
        robot.update();
    }
}
