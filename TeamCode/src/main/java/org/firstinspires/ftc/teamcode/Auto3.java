package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@Autonomous
public class Auto3 extends LinearOpMode {
    Robot robot;
    
    public static final Pose startingPose = new Pose(24*3+12,
                                                     144-((double) 17/2), 180);
    
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        robot.allianceColor = AllianceColor.BLUE;
        robot.spindexer.slots = new Spindexer.Artifact[]{Spindexer.Artifact.PURPLE,
                                              Spindexer.Artifact.PURPLE,
                                              Spindexer.Artifact.PURPLE};
        robot.spindexer.partiallyRotate(0); // Move to first slot
        
        waitForStart();
        
        robot.follower.setCurrentPose(startingPose);
        
        robot.revUpShooterBasedOnDistance();
        
        while (opModeIsActive() && !robot.shooter.isUpToSpeed()) {
            robot.update();
        }
        
        robot.spindexer.rotateToSlot(0);
        sleep(500);
        robot.spindexer.rotateToSlot(1);
        sleep(500);
        robot.spindexer.rotateToSlot(2);

        robot.follower.update();
        blackboard.put("currentPose", robot.follower.getCurrentPose());
    }
}
