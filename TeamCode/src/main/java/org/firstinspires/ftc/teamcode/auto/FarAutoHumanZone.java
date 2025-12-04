package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;

import java.util.Arrays;

@Autonomous
public class FarAutoHumanZone extends Auto2 {
    public Pose startingPose = new Pose(56, 8.5, -90);
    public Pose firePose = new Pose(56, 15, -63);
    
    public Pose humanPlayerZone = new Pose(8, 8.25, 180);
    
    public Pose endPose = new Pose(25, 15, 0);
    
    MotifDetector motifDetector;
    
    @Override
    public void init() {
        super.init();
        robot.preload(new Artifact[]
                          {Artifact.GREEN,
                              Artifact.PURPLE,
                              Artifact.PURPLE});
        
        motifDetector = new MotifDetector(hardwareMap);
        motifDetector.start();
    }
    
    int state = 1;
    
    @Override
    public void start() {
        super.start();
        
        telemetry.addData("startingPose", startingPose);
        telemetry.update();
        
        robot.follower.setCurrentPose(startingPose);
        
        robot.motifPattern = motifDetector.getMotifPattern();
        if (robot.motifPattern == null) {
            robot.motifPattern = new Artifact[]
                {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
        }
        telemetry.addData("pattern", Arrays.deepToString(robot.motifPattern));
        telemetry.update();
    }
    
    @Override
    public void loop() {
        switch (state) {
            case 0:
                robot.follower.drivetrain.zeroPower();
                break;
            case 1:
                goToPose(firePose, Robot.State.REVVING);
                break;
            case 2:
                if (robot.state == Robot.State.IDLE) {
                    state++;
                }
                else {
                    robot.setState(Robot.State.FAST_FIRING);
                }
                break;
            case 3:
                goToPose(humanPlayerZone, Robot.State.LOAD_ARTIFACTS);
                break;
//            case 9:
//                robot.follower.holdPose(endPose);
//                break;
        }
        
        robot.update();
    }
    
    public void goToPose(Pose pose, Robot.State robotState) {
        robot.follower.holdPose(pose);
        robot.setState(robotState);
        
        if (robot.follower.isStoppedAt(pose)) {
            state++;
        }
    }
    
    @Override
    public void stop() {
        super.stop();
        motifDetector.stop();
    }
}
