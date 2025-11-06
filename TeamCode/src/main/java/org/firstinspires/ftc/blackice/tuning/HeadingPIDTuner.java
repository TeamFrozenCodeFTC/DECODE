package org.firstinspires.ftc.blackice.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutine;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Heading PID Tuner", group = "Tuning")
public class HeadingPIDTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize follower with starting pose
        Follower follower = new Follower(hardwareMap, new Pose(0, 0, 0));
        
        // Define target headings (in degrees)
        double[] targetHeadings = {0, 90, 180, -90, -105};
        int currentIndex = 0;
        double targetHeading = Math.toRadians(targetHeadings[currentIndex]);
        
        // Timing variables for switching headings
        ElapsedTime timer = new ElapsedTime();
        double switchInterval = 3.0; // seconds between switches
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance()
            .getTelemetry());
        
        waitForStart();
        timer.reset();
        
        while (opModeIsActive()) {
            follower.update();
            
            // Compute heading correction using your PID
            double turnPower = follower.drivePowerController.computeHeadingCorrectionPower(
                targetHeading,
                follower.getMotionState()
            );
            
            // Apply heading correction only (no translation)
            follower.drivetrain.followVector(Vector.ZERO, turnPower);
            
            // Switch target heading every few seconds
            if (timer.seconds() > switchInterval) {
                currentIndex = (currentIndex + 1) % targetHeadings.length;
                targetHeading = Math.toRadians(targetHeadings[currentIndex]);
                timer.reset();
            }
            
            telemetry.addData("Current Heading (deg)", Math.toDegrees(follower.getCurrentPose().getHeading()));
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Turn Power", turnPower);
            telemetry.addData("Heading Error (deg)",
                              Math.toDegrees(AngleUnit.normalizeDegrees(targetHeading - follower.getCurrentPose().getHeading())));
            telemetry.update();
        }
    }
}
