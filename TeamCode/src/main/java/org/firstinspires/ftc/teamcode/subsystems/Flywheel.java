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
    
//    // PIDFCoefficients(p=10.000000 i=3.000000 d=0.000000 f=0.000000 alg=LegacyPID)
//    public static PIDFCoefficients coefficients = new PIDFCoefficients(20, 3, 0, 13.739);
//
    public static double kP = 20, kI = 3, kD = 0, kS = .1, kV = 13.739;
    
    public PIDFController controller =
        new PIDFController(kP, kI, kD, kS, kV);
    
    double comp = 1;
    
    public void updateVoltComp(double comp) {
        this.comp = comp;
    }

    public void updateCoefficients() {
        controller.setCoefficients(kP, kI, kD, kS, kV);
//        leftMotor.setVelocityPIDFCoefficients(coefficients.p * comp, coefficients.i,
//                                              coefficients.d, coefficients.f * comp);
//        rightMotor.setVelocityPIDFCoefficients(coefficients.p * comp, coefficients.i,
//                                               coefficients.d, coefficients.f * comp);
    }
    
    public void updateRecover() {
        controller.setCoefficients(kP, 0, kD, kS, kV);
//        leftMotor.setVelocityPIDFCoefficients(coefficients.p * comp, 0,
//                                              coefficients.d, coefficients.f * comp);
//        rightMotor.setVelocityPIDFCoefficients(coefficients.p * comp, 0,
//                                               coefficients.d, coefficients.f * comp);
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
    
    public void update(double deltaTime) {
        double rpmDifference = targetRPM - currentTargetRPM;
        double maxStep = ACCELERATION * deltaTime;
        
        if (Math.abs(rpmDifference) > maxStep) {
            currentTargetRPM += Math.copySign(maxStep, rpmDifference);
        } else {
            currentTargetRPM = targetRPM;
        }
        
        double error = currentTargetRPM - getRpm();
        if (error > 500) {
            //controller.reset();
            controller.setCoefficients(kP, 0, kD, kS, kV);
        }
        else {
            updateCoefficients();
        }
        
//        double ticksPerSecond = rpmToTicksPerSecond(currentTargetRPM);
//        rightMotor.setVelocity(ticksPerSecond);
//        leftMotor.setVelocity(ticksPerSecond);
        double power = controller.computeCorrection(error, deltaTime);
        rightMotor.setPower(power);
        leftMotor.setPower(power);
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
