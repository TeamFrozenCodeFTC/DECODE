package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.blackice.util.Logger;

@Config
public class Launcher {
    public final DcMotorEx rightMotor;
    public final DcMotorEx leftMotor;
    
    private double targetRPM = 0;
    private double currentTargetRPM = 0;
    
    public final int TICKS_PER_REV = 28;
    public final double ACCELERATION = 3000; // rpm per second

    public static PIDFCoefficients coefficients = new PIDFCoefficients(45, 5, 15, 187);

    public void updateCoefficients() {
        leftMotor.setVelocityPIDFCoefficients(coefficients.p, coefficients.i,
                                              coefficients.d, coefficients.f);
        rightMotor.setVelocityPIDFCoefficients(coefficients.p, coefficients.i,
                                               coefficients.d, coefficients.f);
    }
    
    public Launcher(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");
        
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        Logger.info("default pid", leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
    }
    
    public boolean rpmDropped() {
        return getRpm() < getTargetRPM() * 0.75;
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
        double rpm = 13.5 * distanceToGoal + 2373;
        setRPM(rpm);
    }
    
    public void update(double deltaTime) {
        //updateCoefficients();
        
        double rpmDifference = targetRPM - currentTargetRPM;
        double maxStep = ACCELERATION * deltaTime;
        
        if (Math.abs(rpmDifference) > maxStep) {
            currentTargetRPM += Math.copySign(maxStep, rpmDifference);
        } else {
            currentTargetRPM = targetRPM;
        }
        
        double ticksPerSecond = rpmToTicksPerSecond(currentTargetRPM);
        rightMotor.setVelocity(ticksPerSecond);
        leftMotor.setVelocity(ticksPerSecond);
    }

    public double getTicksPerSecond() {
        return (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2;
    }
    
    public double getRpm() {
        return ticksPerSecondToRpm(getTicksPerSecond());
    }

    public boolean isUpToSpeed() {
        return targetRPM > 0 && Math.abs(getRpm() - targetRPM) < 50;
    }
    
    public void uptake() {
        setRPM(-435);
    }
    
    public void stop() {
        setRPM(0);
    }
}
