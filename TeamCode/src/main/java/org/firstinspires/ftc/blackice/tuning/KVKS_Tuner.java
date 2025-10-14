package org.firstinspires.ftc.blackice.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.blackice.core.follower.Follower;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class KVKS_Tuner extends LinearOpMode {
    // Keep these small so you don’t need much space
    public static double[] TEST_POWERS = {0.25, 0.4, 0.55};
    public static double SETTLE_TIME = 0.6;  // s to let velocity stabilize
    public static double SAMPLE_TIME = 0.35; // s to average speed samples
    public static long   PAUSE_MS    = 200;  // brief pause between bursts
    
    @Override
    public void runOpMode() {
        Follower follower = new Follower(hardwareMap);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        
        waitForStart();
        
        // Lock heading so we don’t drift sideways while open-looping forward speed
        double headingLock = follower.getMotionState().heading;
        
        List<Double> y_power = new ArrayList<>(); // dependent variable
        List<Double> x1_sign = new ArrayList<>(); // predictor 1: direction sign
        List<Double> x2_sV   = new ArrayList<>(); // predictor 2: sign * speedMagnitude
        
        for (double p : TEST_POWERS) {
            // forward burst
            runBurst(follower, p, y_power, x1_sign, x2_sV);
            // reverse burst
            runBurst(follower, -p, y_power, x1_sign, x2_sV);
        }
        
        follower.drivetrain.zeroPowerBrakeMode();
        
        // Multiple linear regression: power ≈ kS * sign + kV * (sign * speedMag)
        // Build normal equations A * theta = b for theta = [kS, kV]^T
        double s11 = 0, s12 = 0, s22 = 0;
        double b1 = 0, b2 = 0;
        
        for (int i = 0; i < y_power.size(); i++) {
            double s = x1_sign.get(i);
            double sv = x2_sV.get(i);
            double pwr = y_power.get(i);
            
            s11 += s * s;      // = number of samples because s ∈ {-1, +1}
            s12 += s * sv;
            s22 += sv * sv;
            
            b1  += pwr * s;
            b2  += pwr * sv;
        }
        
        double det = s11 * s22 - s12 * s12;
        double kS = ( b1 * s22 - s12 * b2) / det;
        double kV = (-b1 * s12 + s11 * b2) / det;
        
        telemetry.addLine("===== kS / kV Tuning Results =====");
        telemetry.addData("kS", kS);
        telemetry.addData("kV", kV);
        telemetry.addLine("Samples:");
        for (int i = 0; i < y_power.size(); i++) {
            telemetry.addData("[%d] power", String.valueOf(i), y_power.get(i));
            telemetry.addData("     sign",  x1_sign.get(i));
            telemetry.addData("     |v|",  Math.abs(x2_sV.get(i))); // just for visibility
        }
        telemetry.update();
        
        while (opModeIsActive()) idle();
    }
    
    private void runBurst(
        Follower follower,
        double power,
        List<Double> y_power,
        List<Double> x1_sign,
        List<Double> x2_sV
    ) {
        // Command forward vector with heading hold
        follower.drivetrain.followVector(Vector.FORWARD.times(power), 0);
        
        // Let velocity settle
        double t0 = getRuntime();
        while (opModeIsActive() && getRuntime() - t0 < SETTLE_TIME) {
            follower.update();
        }
        
        // Sample speed MAGNITUDE and average
        double sumSpeedMag = 0.0;
        int samples = 0;
        t0 = getRuntime();
        while (opModeIsActive() && getRuntime() - t0 < SAMPLE_TIME) {
            follower.update();
            double speedMag = Math.abs(follower.getMotionState().speed); // <- un-signed speed
            sumSpeedMag += speedMag;
            samples++;
        }
        double avgSpeedMag = (samples > 0) ? (sumSpeedMag / samples) : 0.0;
        
        // Record regression rows
        double sign = Math.signum(power);             // use COMMANDED direction for sign
        y_power.add(power);
        x1_sign.add(sign);
        x2_sV.add(sign * avgSpeedMag);                // sign * speedMagnitude
        
        // brief stop before switching direction
        follower.drivetrain.zeroPowerBrakeMode();
        sleep(PAUSE_MS);
    }
}
