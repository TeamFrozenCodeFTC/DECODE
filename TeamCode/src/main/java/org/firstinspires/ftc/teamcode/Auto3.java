//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.blackice.util.geometry.Pose;
//import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
//
//@Autonomous
//public class Auto3 extends Auto2 {
//    public static final Pose startingPose = new Pose(24*3+12,
//                                                     144-((double) 17/2), 180);
//
//    @Override
//    public void runOpMode() {
//
//
//        waitForStart();
//
//        robot.setState(Robot2.State.FIRING);
//
//        robot.follower.setCurrentPose(startingPose);
//
//        robot.revUpShooterBasedOnDistance();
//
//        while (opModeIsActive() && !robot.shooter.isUpToSpeed()) {
//            robot.update();
//        }
//
//        robot.spindexer.rotateToSlot(0);
//        sleep(500);
//        robot.spindexer.rotateToSlot(1);
//        sleep(500);
//        robot.spindexer.rotateToSlot(2);
//
//        robot.follower.update();
//        blackboard.put("currentPose", robot.follower.getCurrentPose());
//    }
//
//    @Override
//    public void init() {
//        super.init();
//        robot.allianceColor = AllianceColor.BLUE;
//        robot.spindexer.slots = new Spindexer.Artifact[]{Spindexer.Artifact.PURPLE,
//            Spindexer.Artifact.PURPLE,
//            Spindexer.Artifact.PURPLE};
//        robot.spindexer.partiallyRotate(0); // Move to first slot
//    }
//
//    @Override
//    public void init_loop() {
//        super.init_loop();
//    }
//
//
//    @Override
//    public void start() {
//        super.start();
//    }
//
//    @Override
//    public void loop() {
//
//    }
//}
