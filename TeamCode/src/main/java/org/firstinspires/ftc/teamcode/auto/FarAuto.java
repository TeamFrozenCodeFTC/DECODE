//package org.firstinspires.ftc.teamcode.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.blackice.util.geometry.Pose;
//import org.firstinspires.ftc.teamcode.Artifact;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;
//
//import java.util.Arrays;
//
//@Autonomous
//public class FarAuto extends Auto {
//    public Pose startingPose = new Pose(56, 8.5, -90);
//    public Pose firePose = new Pose(56, 15, -63);
//
//    public Pose endPose = new Pose(25, 15, 0);
//
//    MotifDetector motifDetector;
//
//    @Override
//    public void init() {
//        super.init();
//        robot.preload(new Artifact[]
//                  {Artifact.GREEN,
//                      Artifact.PURPLE,
//                      Artifact.PURPLE});
//
//        motifDetector = new MotifDetector(hardwareMap);
//        motifDetector.start();
//    }
//
//    int state = 1;
//
//    @Override
//    public void start() {
//        super.start();
//
//        telemetry.addData("startingPose", startingPose);
//        telemetry.update();
//
//        robot.follower.setCurrentPose(startingPose);
//
//        Robot.motifPattern = motifDetector.getMotifPattern();
//        if (Robot.motifPattern == null) {
//            Robot.motifPattern = new Artifact[]
//                {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
//        }
//        telemetry.addData("pattern", Arrays.deepToString(Robot.motifPattern));
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        switch (state) {
//            case 0:
//                robot.follower.drivetrain.zeroPower();
//                break;
//            case 7:
//            case 4:
//            case 1:
//                robot.paddles.close();
//                goToPose(firePose, Robot.State.REVVING);
//                break;
//            case 8:
//            case 5:
//            case 2:
//                if (robot.state == Robot.State.IDLE) {
//                    state++;
//                }
//                else {
//                    robot.setState(Robot.State.FIRING);
//                }
//                robot.follower.holdPose(firePose);
//                break;
//            case 3:
//                if (pickupArtifactGroup(36)) {
//                    robot.paddles.close();
//                    state++;
//                }
//                break;
//            case 6:
//                if (pickupArtifactGroup(60)) {
//                    robot.paddles.close();
//                    state++;
//                }
//                break;
//            case 9:
//                robot.follower.holdPose(endPose);
//                break;
//        }
//
//        if (time > 30 - 3) {
//            state = 9;
//        }
//
//        robot.update();
//    }
//
////    public void goToPose(Pose pose, Robot.State robotState) {
////        robot.follower.holdPose(pose);
////        robot.setState(robotState);
////
////        if (robot.follower.isStoppedAt(pose)) {
////            state++;
////        }
////    }
//
//    @Override
//    public void stop() {
//        super.stop();
//        motifDetector.stop();
//    }
//}
