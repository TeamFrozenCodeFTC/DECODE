package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Haptics;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.testing.Menu;

import java.util.Arrays;

public class TeleOps extends OpMode {
    Robot robot;
    
    private int motifIndex = 0;
    
    private final Artifact[][] MOTIF_PATTERNS = {
        {Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN}, // PPG
        {Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE}, // PGP
        {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE}  // GPP
    };
    
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        robot.follower.drivetrain.zeroPowerBrakeMode();
        
        for (int i = 0; i < MOTIF_PATTERNS.length; i++) {
            if (Arrays.equals(Robot.motifPattern, MOTIF_PATTERNS[i])) {
                motifIndex = i;
                break;
            }
        }
        
        if (Robot.artifacts != null) {
            robot.spindexer.artifacts = Robot.artifacts;
        }
        else {
            robot.spindexer.artifacts = new Artifact[]
                {Artifact.NONE, Artifact.NONE, Artifact.NONE};
        }
    }
    
    @Override
    public void init_loop() {
        if (gamepad1.triangleWasPressed()) {
            Robot.allianceColor = (AllianceColor.BLUE == Robot.allianceColor)
                ? AllianceColor.RED
                : AllianceColor.BLUE;
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        if (gamepad1.squareWasPressed()) {
            motifIndex = (motifIndex + 1) % MOTIF_PATTERNS.length;
            Robot.motifPattern = MOTIF_PATTERNS[motifIndex];
            gamepad1.rumble(Haptics.CONFIRM);
        }
        
        telemetry.addData("Alliance Color (△)", Robot.allianceColor);
        telemetry.addData("Motif Pattern (□)", Arrays.deepToString(Robot.motifPattern));
        telemetry.addData("position", Robot.currentPose);
        telemetry.update();
    }
    
    @Override
    public void start() {
        robot.follower.setCurrentPose(Robot.allianceColor.getHumanResetZone());
        
        if (Robot.currentPose != null) {
            robot.follower.setCurrentPose(Robot.currentPose);
        }
        
        robot.spindexer.rotateToSlot(0);
        robot.intakeRamp.uptake();
        robot.paddles.open();
    }
    
    @Override
    public void loop() {
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
            telemetry.addData("current rpm", "%.2f", robot.flywheel.getRpm());
            telemetry.addData("target rpm", "%.2f", robot.flywheel.getTargetRPM());
            telemetry.addData("numOfArtifacts", robot.spindexer.getNumberOfArtifacts());
            telemetry.addData("artifacts", Arrays.deepToString(robot.spindexer.artifacts));
            telemetry.addData("spindexer index", robot.spindexer.currentSlotIndex);
            telemetry.addData("spindexerIsRotating", robot.spindexerIsRotating);
            telemetry.addData("isUpToSpeed", robot.flywheel.isUpToSpeed());
            telemetry.addData("position", robot.follower.getCurrentPose());
            telemetry.addData("distanceToGoal",
                              Robot.allianceColor.getGoalPosition()
                                  .distanceTo(
                                      robot.follower.getCurrentPose().getPosition()));
            telemetry.addData("firedArtifacts", robot.firedArtifacts);
            telemetry.addData("isLookingAtGoal()", robot.isLookingAtGoal());
            telemetry.addData("goal angle", robot.getAngleToGoal());
            telemetry.addData("current angle",
                              robot.follower.localizer.getPose().getHeading());
            telemetry.addData("waitingForDrop", robot.spindexer.waitingForDrop);
            telemetry.update();
        }
        
        robot.update();
    }
}
