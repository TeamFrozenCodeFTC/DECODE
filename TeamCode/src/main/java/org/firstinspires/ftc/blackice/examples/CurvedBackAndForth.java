package org.firstinspires.ftc.blackice.examples;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.paths.HeadingInterpolator;
import org.firstinspires.ftc.blackice.core.paths.Path;
import org.firstinspires.ftc.blackice.util.Logger;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group="Black-Ice Examples")
public class CurvedBackAndForth extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap, new Pose(0, 0, 45));
        
        waitForStart();
        
        while (opModeIsActive()) {
            Logger.debug("startPose", follower.getCurrentPose());
            Logger.debug("startPose", follower.pathRoutineBuilder().startingPose);
            
            PathRoutine path1 = follower.pathRoutineBuilder()
                .curveTo(new Vector(24, 12), new Vector(48, 0))
                .stop()
                .build();
            
            PathRoutine path2 = follower.pathRoutineBuilder()
                .withStartPose(path1.endPose)
                .curveTo(new Vector(24, 12), new Vector(0, 0))
                .withHeadingInterpolator(HeadingInterpolator.tangent.backwards())
                .stop()
                .build();
            
            Logger.debug("midpoint",
                         ((Path) path2.getStep(0)).geometry.computeClosestPathPointTo(new Vector(36, 13), 0.5).point);
            Logger.debug("endPose2", path2.endPose);
            Logger.debug("endPose1", path1.endPose);
            
            // --- follow first path ---
            follower.follow(path1);
            
            List<Double> pathXs = new ArrayList<>();
            List<Double> pathYs = new ArrayList<>();
            List<Double> robotXs = new ArrayList<>();
            List<Double> robotYs = new ArrayList<>();
            
            while (opModeIsActive() && follower.isInProgress()) {
                Logger.debug("currentHeading", follower.getCurrentPose().getHeading());
                
                follower.update();
                
                telemetry.addData("State", follower.getFollowingState());
                telemetry.update();
                
                Vector pathPoint = follower.getClosestPathPoint().point;
                Vector robotPosition = follower.getMotionState().position;
                
                // record curve + trail
                pathXs.add(pathPoint.getX());
                pathYs.add(pathPoint.getY());
                robotXs.add(robotPosition.getX());
                robotYs.add(robotPosition.getY());
                
                // dashboard packet
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay()
                    // draw curve from path points
                    .setStroke("red")
                    .strokePolyline(
                        pathXs.stream().mapToDouble(Double::doubleValue).toArray(),
                        pathYs.stream().mapToDouble(Double::doubleValue).toArray()
                    )
                    
                    // draw robot trail
                    .setStroke("blue")
                    .strokePolyline(
                        robotXs.stream().mapToDouble(Double::doubleValue).toArray(),
                        robotYs.stream().mapToDouble(Double::doubleValue).toArray()
                    )
                    
                    // robot’s current position
                    .setFill("green")
                    .fillCircle(robotPosition.getX(), robotPosition.getY(), 3);
                
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
            
                        // --- follow second path ---
                        follower.follow(path2);

                        // reset lists for new routine
                        pathXs.clear();
                        pathYs.clear();
                        robotXs.clear();
                        robotYs.clear();

                        while (opModeIsActive() && follower.isInProgress()) {
                            follower.update();

                            Vector pathPoint = follower.getClosestPathPoint().point;
                            Vector robotPosition = follower.getMotionState().position;

                            pathXs.add(pathPoint.getX());
                            pathYs.add(pathPoint.getY());
                            robotXs.add(robotPosition.getX());
                            robotYs.add(robotPosition.getY());

                            TelemetryPacket packet = new TelemetryPacket();
                            packet.fieldOverlay()
                                .setStroke("red")
                                .strokePolyline(
                                    pathXs.stream().mapToDouble(Double::doubleValue).toArray(),
                                    pathYs.stream().mapToDouble(Double::doubleValue).toArray()
                                )
                                .setStroke("blue")
                                .strokePolyline(
                                    robotXs.stream().mapToDouble(Double::doubleValue).toArray(),
                                    robotYs.stream().mapToDouble(Double::doubleValue).toArray()
                                )
                                .setFill("green")
                                .fillCircle(robotPosition.getX(), robotPosition.getY(), 3);

                            FtcDashboard.getInstance().sendTelemetryPacket(packet);
                        }
                    }
        
    }
}
