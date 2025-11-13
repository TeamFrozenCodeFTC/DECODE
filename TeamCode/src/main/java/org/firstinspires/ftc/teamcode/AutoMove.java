package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@Autonomous
public class AutoMove extends Auto2 {
    public Pose startingPose = new Pose(24*2+12,
                                               144-((double) 17/2), 0);
    public Pose artifactGroup1Pose = new Pose(42, 84, 180);
    
    @Override
    public void init() {
        super.init();
        robot.spindexer.slots = new Spindexer.Artifact[]
            {Spindexer.Artifact.PURPLE,
                Spindexer.Artifact.PURPLE,
                Spindexer.Artifact.PURPLE};
    }
    
    public enum State {
        ONE,
        IDLE
    }
    
    State state = State.ONE;
    
    @Override
    public void start() {
        super.start();
        
        
        telemetry.addData("startingPose", startingPose);
        telemetry.update();
        
        robot.follower.setCurrentPose(startingPose);
    }
    
    public void loop() {
        robot.follower.update();
        
        switch (state) {
            case ONE:
                robot.follower.holdPose(artifactGroup1Pose);
                
                if (robot.follower.isStoppedAt(artifactGroup1Pose)) {
                    state = State.IDLE;
                }
                break;
            case IDLE:
                robot.follower.drivetrain.zeroPower();
                break;
        }
    }
}
