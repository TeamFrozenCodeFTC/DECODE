//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.blackice.util.geometry.Pose;
//import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
//
//import java.util.Arrays;
//
//@Autonomous
//public class FarAuto extends Auto2 {
//    public Pose startingPose = new Pose(60, 8.5, -90);
//    public Pose aprilTagPose = new Pose(60, 24, -90);
//    public Pose fireFirst3 = new Pose(60, 24, -63);
//    public Pose pickupFirst = new Pose(42, 84, 180);
//
//    AprilTag aprilTag;
//
//    @Override
//    public void init() {
//        super.init();
//        robot.spindexer.slots = new Spindexer.Artifact[]
//            {Spindexer.Artifact.PURPLE,
//                Spindexer.Artifact.PURPLE,
//                Spindexer.Artifact.PURPLE};
//        aprilTag = new AprilTag(hardwareMap);
//    }
//
//    int state = 0;
//
//    @Override
//    public void start() {
//        super.start();
//
//
//        telemetry.addData("startingPose", startingPose);
//        telemetry.update();
//
//        robot.follower.setCurrentPose(startingPose);
//    }
//
//    @Override
//    public void loop() {
//        robot.follower.update();
//
//        switch (state) {
//            case 0:
//                robot.follower.holdPose(aprilTagPose);
//
//                if (robot.follower.isStoppedAt(aprilTagPose)) {
//                    aprilTag.start();
//                    Spindexer.Artifact[] pattern = aprilTag.getPattern(); // replace Object with
//                    // actual type if you
//                    // know it
//                    telemetry.addData("pattern",
//                                      (pattern != null)
//                                          ? (pattern.getClass().isArray() ? Arrays.deepToString((Object[]) pattern) : pattern)
//                                          : "null"
//                    );
//                    telemetry.update();
//                    aprilTag.stop();
//                    state = 1;
//                }
//                break;
//            case 1:
//                robot.follower.holdPose(fireFirst3);
//                robot.setState(Robot.State.REVVING);
//
//                if (robot.follower.isStoppedAt(fireFirst3)) {
//                    state = 3;
//                }
//            case 3:
//                robot.setState(Robot.State.FIRING);
//
//                if (robot.state == Robot.State.IDLE) {
//                    state = 4;
//                }
//
//                break;
//            case 4:
//                robot.follower.holdPose(pickupFirst);
//
//                if (robot.follower.isStoppedAt(pickupFirst)) {
//                    state = 2;
//                } // .484
//                break;
//            case 2:
//                robot.follower.drivetrain.zeroPower();
//                break;
//        }
//    }
//}
