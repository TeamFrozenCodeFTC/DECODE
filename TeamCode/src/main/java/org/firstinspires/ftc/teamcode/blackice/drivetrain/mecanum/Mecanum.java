package org.firstinspires.ftc.teamcode.blackice.drivetrain.mecanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.blackice.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

class Mecanum implements Drivetrain {
    public final double strafingEffortMultiplier;
    
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;
    
    public Mecanum(HardwareMap map, MecanumConfig config) {
        frontLeft  = map.get(DcMotorEx.class, config.frontLeftName);
        backLeft   = map.get(DcMotorEx.class, config.backLeftName);
        frontRight = map.get(DcMotorEx.class, config.frontRightName);
        backRight  = map.get(DcMotorEx.class, config.backRightName);
        
        frontLeft.setDirection(config.frontLeftDirection);
        backLeft.setDirection(config.backLeftDirection);
        frontRight.setDirection(config.frontRightDirection);
        backRight.setDirection(config.backRightDirection);
        
        strafingEffortMultiplier = config.maxForwardSpeed / config.maxStrafeSpeed;
    }
    
    private Vector adjustDirectionalEffort(Vector input) {
        return new Vector(
            input.getX(),
            input.getY() * strafingEffortMultiplier
        );
    }
    
    @Override
    public void followVector(Vector robotVector, double turnPower) {
        Vector v = adjustDirectionalEffort(robotVector);
        
        double upRight  = -v.getY() + v.getX();  // FL, BR
        double downLeft = -v.getY() - v.getX();  // BL, FR
        
        double fl = upRight  - turnPower;
        double bl = downLeft + turnPower;
        double fr = downLeft - turnPower;
        double br = upRight  + turnPower;
        
        double max = Math.max(
            Math.max(Math.abs(fl), Math.abs(bl)),
            Math.max(Math.abs(fr), Math.abs(br))
        );
        
        if (max > 1.0) {
            double scale = 1.0 / max;
            fl *= scale;
            bl *= scale;
            fr *= scale;
            br *= scale;
        }
        
        // Apply motor power
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }
    
    @Override
    public void zeroPowerBrakeMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    @Override
    public void zeroPower() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
    
    @Override
    public void zeroPowerFloatMode() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
