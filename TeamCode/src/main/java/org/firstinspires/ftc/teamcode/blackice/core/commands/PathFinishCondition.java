package org.firstinspires.ftc.teamcode.blackice.core.commands;

import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

@FunctionalInterface
public interface PathFinishCondition {
    boolean isFinished(Follower follower, Pose endPose);
}
