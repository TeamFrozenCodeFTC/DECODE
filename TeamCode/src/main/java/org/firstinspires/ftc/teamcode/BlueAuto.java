package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;

@Autonomous
public class BlueAuto extends LinearOpMode {
    Robot robot;
    
    public static final Pose startingPose = new Pose(24*3+12,
                                                     144-((double) 17/2), 180);
    
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        robot.allianceColor = AllianceColor.BLUE;
        
        waitForStart();
        
        robot.follower.setCurrentPose(startingPose);
        
        robot.revUpShooterBasedOnDistance();
        
        while (opModeIsActive() && !robot.shooter.isUpToSpeed()) {
            robot.update();
        }
        
        robot.intake.intake();
        
        sleep(1000);
        
        robot.stopIntake();
        
        robot.follower.update();
        blackboard.put("currentPose", robot.follower.getCurrentPose());
    }
}
