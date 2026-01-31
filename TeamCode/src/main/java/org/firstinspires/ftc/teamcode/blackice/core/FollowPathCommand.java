package org.firstinspires.ftc.teamcode.blackice.core;

import org.firstinspires.ftc.teamcode.blackice.core.commands.Command;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathPoint;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

public class FollowPathCommand extends Command  {
    PathGeometry pathGeometry;
    HeadingInterpolator headingInterpolator;
    Follower follower;
    double lastTValue = 0;
    
    public FollowPathCommand(PathGeometry pathGeometry,
                             HeadingInterpolator headingInterpolator) {
        this.pathGeometry = pathGeometry;
        this.headingInterpolator = headingInterpolator;
    }
    
    @Override
    public void onStart() {
        lastTValue = 0;
    }
    
    @Override
    public void update() {
        Vector position = follower.getCurrentPose().getPosition();
        
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
            ); // * normalAuthority (lower for swerve and tank)
        
        double tangentPower =
            follower.positionalController.computeOutput(
                closest.distanceRemaining,
                velocity.dot(tangent)
            );

        double headingPower =
            follower.computeHeadingCorrectionPower(
                headingInterpolator.interpolate(closest)
            );
        
        double powerBudget = 1.0;
        
        double normalUsed = allocatePower(normalPower, powerBudget);
        powerBudget -= Math.abs(normalUsed);
        
        double headingUsed = allocatePower(headingPower, powerBudget);
        powerBudget -= Math.abs(headingUsed);
        
        double tangentUsed = allocatePower(tangentPower, powerBudget);
        
        Vector drivePower =
            normal.times(normalUsed)
                .plus(tangent.times(tangentUsed));
        
        follower.followFieldVector(drivePower, headingPower);
    }
    
    public double allocatePower(double requested, double budget) {
        return Math.copySign(
            Math.min(Math.abs(requested), budget),
            requested
        );
    }
    
    @Override
    public boolean onIsFinished() {
        return follower.isWithinBraking(pathGeometry.getEndPathPoint().point);
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
