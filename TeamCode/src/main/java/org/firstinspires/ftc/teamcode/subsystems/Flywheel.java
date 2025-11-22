package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.blackice.core.control.Feedforward;
import org.firstinspires.ftc.blackice.core.control.PIDFController;

@Config
public class Flywheel {
    public final DcMotorEx rightMotor;
    public final DcMotorEx leftMotor;
    
    private double targetRPM = 0;
    private double currentTargetRPM = 0;
    
    public final int TICKS_PER_REV = 28;
    public final double ACCELERATION = 3000; // rpm per second
    
    private static final int ACCEL_SAMPLES = 10; // adjust: 10 samples at 20ms = 200ms history
    private final double[] accelWindow = new double[ACCEL_SAMPLES];
    private int accelIndex = 0;
    private boolean accelFilled = false;
    
    private double lastRpm = 0;
    
    
    // http://192.168.43.1:8080/dash
    // PIDFCoefficients(p=10.000000 i=3.000000 d=0.000000 f=0.000000 alg=LegacyPID)
    public static PIDFCoefficients coefficients = new PIDFCoefficients(20, 3, 0, 13.739);

//    public static double kP = 20, kI = 3, kD = 0, kS = .1, kV = 13.739;
//    public static double kP = 0, kI = 0, kD = 0, kS = 0, kV = 0;
    public static double disableIAt = 500;
    
    public static double kI = 0;
    
    // 0.000185
    
    
    double comp = 1;
    
    public void updateVoltComp(double comp) {
        this.comp = comp;
    }

    public void updateCoefficients() {
      //  controller.setCoefficients(kP, kI, kD, kS, kV);
        leftMotor.setVelocityPIDFCoefficients(coefficients.p, coefficients.i,
                                              coefficients.d, coefficients.f);
        rightMotor.setVelocityPIDFCoefficients(coefficients.p, coefficients.i,
                                               coefficients.d, coefficients.f);
    }
    
    public void updateRecover() {
       /// controller.setCoefficients(kP, 0, kD, kS, kV);
        leftMotor.setVelocityPIDFCoefficients(coefficients.p * comp, 0,
                                              coefficients.d, coefficients.f * comp);
        rightMotor.setVelocityPIDFCoefficients(coefficients.p * comp, 0,
                                               coefficients.d, coefficients.f * comp);
    }
    
    public Flywheel(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    
    public boolean rpmDropped() {
        return getRpm() < getTargetRPM() * 0.90;
    }
    
    public double getTargetRPM() {
        return targetRPM;
    }
    
    public double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV / 60;
    }
    
    public double ticksPerSecondToRpm(double ticksPerSecond) {
        return ticksPerSecond / TICKS_PER_REV * 60;
    }
    
    public void setRPM(double rpm) {
        targetRPM = rpm;
    }
    
    public void setRpmFromDistance(double distanceToGoal) {
        double rpm = 13.5 * distanceToGoal + 2373; // from like 10 data points
        setRPM(rpm);
    }
    
    double totalError = 0;
    
    public void update(double deltaTime) {
        double currentRpm = getRpm();
        
        // --- acceleration tracking ---
        double accel = (currentRpm - lastRpm) / deltaTime;  // rpm/sec
        accelWindow[accelIndex++] = accel;
        
        if (accelIndex >= ACCEL_SAMPLES) {
            accelIndex = 0;
            accelFilled = true;
        }
        
        lastRpm = currentRpm;
        
        // --- rest of your update logic ---
        double rpmDifference = targetRPM - currentTargetRPM;
        double maxStep = ACCELERATION * deltaTime;
        
        if (Math.abs(rpmDifference) > maxStep) {
            currentTargetRPM += Math.copySign(maxStep, rpmDifference);
        } else {
            currentTargetRPM = targetRPM;
        }
        
        double ticksPerSecond = rpmToTicksPerSecond(currentTargetRPM);
        
        double error = currentTargetRPM - currentRpm;
        if (Math.abs(error) > disableIAt) {
            totalError += error;
            ticksPerSecond -= totalError * kI;
        }
        
        updateCoefficients();
        rightMotor.setVelocity(ticksPerSecond);
        leftMotor.setVelocity(ticksPerSecond);
    }
    
    private double getAverageAcceleration() {
        int count = accelFilled ? ACCEL_SAMPLES : accelIndex;
        if (count == 0) return Double.MAX_VALUE;
        
        double sum = 0;
        for (int i = 0; i < count; i++) {
            sum += accelWindow[i];
        }
        return sum / count;
    }
    
    public boolean isUpToSpeed() {
        if (targetRPM <= 0) return false;
        
        double rpmError = Math.abs(getRpm() - targetRPM);
        double avgAccel = Math.abs(getAverageAcceleration());
        
        final double MAX_ERROR = 50;       // rpm
        final double MAX_ACCEL = 100;      // rpm/sec
        
        return rpmError < MAX_ERROR;
    }
    
    public double getTicksPerSecond() {
        return (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2;
    }
    
    public double getRpm() {
        return ticksPerSecondToRpm(getTicksPerSecond());
    }

//    public boolean isUpToSpeed() {
//        //
//        return targetRPM > 0 && Math.abs(getRpm() - targetRPM) < 50;
//    }
    
    public void uptake() {
        setRPM(-435);
    }
    
    public void stop() {
        setRPM(0);
    }
}
