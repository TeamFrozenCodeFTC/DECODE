package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;

@Autonomous
public class AprilAuto extends Auto2 {
    public Pose startingPose = new Pose(60, 135.5, -90);
    public Pose aprilTagPose = new Pose(60, 88, -90);
    public Pose fire3 = new Pose(60, 88, -43);
    public AprilTag aprilTag;
    
    @Override
    public void init() {
        super.init();
        robot.spindexer.slots = new Spindexer.Artifact[]
            {Spindexer.Artifact.PURPLE,
                Spindexer.Artifact.PURPLE,
                Spindexer.Artifact.PURPLE};
        aprilTag = new AprilTag(hardwareMap);
    }
    
    public enum State {
        ONE,
        TWO,
        THREE,
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
    
    @Override
    public void loop() {
        robot.follower.update();
        
        switch (state) {
            case ONE:
                robot.follower.holdPose(aprilTagPose);
                
                if (robot.follower.isStoppedAt(aprilTagPose)) {
                    aprilTag.start();
                    Spindexer.Artifact[] pattern = aprilTag.getPattern(); // replace Object with
                    // actual type if you
                    // know it
                    telemetry.addData("pattern",
                                      (pattern != null)
                                          ? (pattern.getClass().isArray() ? Arrays.deepToString((Object[]) pattern) : pattern)
                                          : "null"
                    );
                    telemetry.update();
                    aprilTag.stop();
                    state = State.TWO;
                }
                break;
            case TWO:
                robot.follower.holdPose(fire3);
                
                if (robot.follower.isStoppedAt(fire3)) {
                    state = State.IDLE;
                }
            case IDLE:
                robot.follower.drivetrain.zeroPower();
                break;
        }
    }
}
