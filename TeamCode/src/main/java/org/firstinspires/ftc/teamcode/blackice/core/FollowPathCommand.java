package org.firstinspires.ftc.teamcode.blackice.core;

import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.core.commands.PathFinishCondition;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathPoint;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

public class FollowPathCommand extends Command  {
    final PathGeometry pathGeometry;
    final HeadingInterpolator headingInterpolator;
    final PathFinishCondition finishCondition;
    final Follower follower;
    double lastTValue = 0;
    
    Pose endPose;
    
    public FollowPathCommand(PathGeometry pathGeometry,
                             HeadingInterpolator headingInterpolator,
                             PathFinishCondition finishCondition,
                             Follower follower) {
        this.pathGeometry = pathGeometry;
        this.headingInterpolator = headingInterpolator;
        this.finishCondition = finishCondition;
        this.follower = follower;
        
        endPose = new Pose(
            pathGeometry.getEndPathPoint().point,
            Math.toDegrees(headingInterpolator.interpolate(pathGeometry.getEndPathPoint())
        ));
    }
    
    @Override
    public void onStart() {
        lastTValue = 0;
    }
    
    @Override
    public void update() {
        Vector position = follower.getPosition();
        
        PathPoint closest =
            pathGeometry.computeClosestPathPointTo(position, lastTValue);
        lastTValue = closest.tValue;
        
        Vector tangent = Vector.fromPolar(1, closest.tangent);
        Vector normal = tangent.perpendicularLeft();
        
        Vector velocity = follower.getVelocity();
        
        double normalError = closest.point.minus(position).dot(normal);
        double normalPower =
            follower.positionalController.computeOutput(
                normalError,
                velocity.dot(normal)
            ); // * normalAuthority (lower for swerve and tank cause no normal correction)
        
        double distanceToEnd;
        if (closest.distanceRemaining == 0) {
            distanceToEnd =
                pathGeometry.getEndPathPoint().point.minus(position).dot(tangent);
        }
        else {
            distanceToEnd = closest.distanceRemaining;
        }
        
        double tangentPower =
            follower.positionalController.computeOutput(
                distanceToEnd,
                velocity.dot(tangent)
            );
    
        double targetHeading = headingInterpolator.interpolate(closest);
        double headingPower =
            follower.computeHeadingCorrectionPower(targetHeading);
        
        double maxMagnitude = 1.0;
        double normalUsed = allocatePower(normalPower, maxMagnitude);
        double remaining =
            Math.sqrt(
                Math.max(
                    0.0,
                    maxMagnitude * maxMagnitude - normalUsed * normalUsed
                )
            );
        double headingUsed = allocatePower(headingPower, remaining);
        remaining =
            Math.sqrt(
                Math.max(
                    0.0,
                    remaining * remaining - headingUsed * headingUsed
                )
            );
        double tangentUsed = allocatePower(tangentPower, remaining);

        Vector drivePower =
            normal.times(normalUsed)
                .plus(tangent.times(tangentUsed));
        
        follower.followFieldVector(drivePower, headingPower);
  
        follower.telemetry.addData("holdPower",
                                   follower.computeHoldPower(pathGeometry.getEndPathPoint().point));
        follower.telemetry.addData("isWithinBraking",
                                   finishCondition.isFinished(follower, endPose));
        follower.telemetry.addData("distanceToEnd", distanceToEnd);
        follower.telemetry.addData("position", position);
        follower.telemetry.addData("closestT", closest.tValue);
        follower.telemetry.addData("normalError", normalError);
        follower.telemetry.addData("normalPower", normalPower);
        follower.telemetry.addData("tangentPower", tangentPower);
        follower.telemetry.addData("headingPower", headingPower);

        follower.telemetry.addData("percentAlongPath", closest.percentAlongPath);
        follower.telemetry.addData("currentPose", follower.getCurrentPose());
        follower.telemetry.addData("endPose", endPose);
        follower.telemetry.addData("drivePower", drivePower);
        follower.telemetry.update();
    }
    
    public double allocatePower(double requested, double budget) {
        return Math.copySign(
            Math.min(Math.abs(requested), budget),
            requested
        );
    }
    
    @Override
    public boolean onIsFinished() {
        return finishCondition.isFinished(follower, endPose);
    }
}


//    double maxMagnitude = 1.0;
//
//    double normalUsed = clamp(normalPower, maxMagnitude);
//
//    double remaining =
//        Math.sqrt(
//            Math.max(
//                0.0,
//                maxMagnitude * maxMagnitude - normalUsed * normalUsed
//            )
//        );
//
//    double tangentUsed = clamp(tangentPower, remaining);
//
