package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;

@Autonomous
public class FarAuto extends Auto2 {
    public Pose startingPose = new Pose(60, 8.5, -90);
    public Pose firePose = new Pose(60, 24, -63);
    
    public Pose pickupGroup1 = new Pose(41, 36, 180);
    public Pose endGroup1 = new Pose(20, 36, 180);
    
    public Pose endPose = new Pose(25, 15, 0);
 
    MotifDetector motifDetector;

    @Override
    public void init() {
        super.init();
        robot.spindexer.slots = new Spindexer.Artifact[]
            {Spindexer.Artifact.GREEN, Spindexer.Artifact.PURPLE, Spindexer.Artifact.PURPLE};
        
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
        telemetry.addData("pattern", Arrays.deepToString(robot.motifPattern));
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                robot.follower.drivetrain.zeroPower();
                break;
            case 5:
            case 1:
                goToPose(firePose, Robot.State.REVVING);
                break;
            case 6:
            case 2:
                if (robot.state == Robot.State.IDLE) {
                    state++;
                }
                else {
                    robot.setState(Robot.State.FIRING);
                }
                break;
            case 3:
                goToPose(pickupGroup1, Robot.State.LOAD_ARTIFACTS);
                break;
            case 4:
                robot.follower.holdPose(endGroup1, 0.2);
                
                if (robot.follower.isStoppedAt(endGroup1)) {
                    state++;
                }
                break;
            case 7:
                robot.follower.holdPose(endPose);
                break;
        }
        
        robot.update(telemetry);
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
        motifDetector.stop();
    }
}
