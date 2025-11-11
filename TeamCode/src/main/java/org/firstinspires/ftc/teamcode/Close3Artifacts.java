package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@Autonomous
public class Close3Artifacts extends Auto2 {
    public static Pose startingPose = new Pose(24*2+12,
                                                144-((double) 17/2), 0);

    @Override
    public void init() {
        super.init();
        robot.allianceColor = AllianceColor.BLUE;
        robot.spindexer.slots = new Spindexer.Artifact[]
            {Spindexer.Artifact.PURPLE,
            Spindexer.Artifact.PURPLE,
            Spindexer.Artifact.PURPLE};
        robot.spindexer.rotateToSlot(-0.5);
    }

    @Override
    public void start() {
        super.start();
        
        
        telemetry.addData("startingPose", startingPose);
        telemetry.update();
        
        robot.follower.setCurrentPose(startingPose);
        robot.setState(Robot.State.FIRING);
    }
}
