package org.firstinspires.ftc.blackice.core.hardware.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

public class Mecanum implements Drivetrain {
    public final double strafingEffortMultiplier;
    public final DcMotorEx[] motors;

    public Mecanum(HardwareMap map, MecanumConfig config) {
        DcMotorEx frontLeft = map.get(DcMotorEx.class, config.frontLeftName);
        DcMotorEx backLeft = map.get(DcMotorEx.class, config.backLeftName);
        DcMotorEx frontRight = map.get(DcMotorEx.class, config.frontRightName);
        DcMotorEx backRight = map.get(DcMotorEx.class, config.backRightName);
        
        frontLeft.setDirection(config.frontLeftDirection);
        backLeft.setDirection(config.backLeftDirection);
        frontRight.setDirection(config.frontRightDirection);
        backRight.setDirection(config.backRightDirection);
        
        this.motors = new DcMotorEx[] { frontLeft, backLeft, frontRight, backRight };
        this.strafingEffortMultiplier = config.maxForwardSpeed / config.maxStrafeSpeed;
    }

    public Vector adjustDirectionalEffort(Vector inputEffort) {
        return new Vector(
            inputEffort.getX(),
            inputEffort.getY() * this.strafingEffortMultiplier
        );
    }
    
    private WheelPowers getTranslationalPowers(Vector robotVector) {
        Vector adjustedVector = adjustDirectionalEffort(robotVector);
        
        double upRightDirection = -adjustedVector.getY() + adjustedVector.getX();
        double downLeftDirection = -adjustedVector.getY() - adjustedVector.getX();
        
        return new WheelPowers(
            upRightDirection, downLeftDirection,
            downLeftDirection, upRightDirection
        );
    }
    
    @Override
    public void followVector(Vector robotVector, double turnPower) {
        WheelPowers translationalPowers = getTranslationalPowers(robotVector);
        WheelPowers rotationalPowers = new WheelPowers(-turnPower, turnPower,
                                                       -turnPower, turnPower);
        
        applyPowers(translationalPowers
            .plus(rotationalPowers)
            .downscaleMaxTo(1)
        );
    }

    public void applyPowers(WheelPowers powers) {
        double[] motorPowers = powers.getPowers();
        for (int i = 0; i < motorPowers.length; i++) {
            motors[i].setPower(motorPowers[i]);
        }
    }
    
    @Override
    public void zeroPowerBrakeMode() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    @Override
    public void zeroPower() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }
    
    @Override
    public void zeroPowerFloatMode() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
