package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.MotifDetector;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;

@Autonomous
public class CloseAprilTagAuto extends Auto2 {
    public Pose startingPose = new Pose(60, 135.5, -90);
    public Pose aprilTagPose = new Pose(60, 88, -90);
    public Pose fire3 = new Pose(60, 88, -43);
    public MotifDetector aprilTag;

    @Override
    public void init() {
        super.init();
        aprilTag = new MotifDetector(hardwareMap);
        
        robot.spindexer.slots = Artifact.getHumanPlayerPattern();
        robot.spindexer.rotateToSlot(2.5);
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
                    Artifact[] pattern = aprilTag.getMotifPattern();
                    telemetry.addData("pattern",
                                      (pattern != null)
                                          ? (pattern.getClass().isArray() ? Arrays.deepToString(
                                          pattern) : pattern)
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
