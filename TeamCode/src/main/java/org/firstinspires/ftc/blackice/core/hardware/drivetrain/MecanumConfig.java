package org.firstinspires.ftc.blackice.core.hardware.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumConfig implements DrivetrainConfig {
    String frontLeftName, backLeftName, frontRightName, backRightName;
    DcMotorSimple.Direction frontLeftDirection, backLeftDirection,
                             frontRightDirection, backRightDirection;

    double maxForwardSpeed;
    double maxStrafeSpeed;
    
    public MecanumConfig() {}
    
    public MecanumConfig frontLeft(String name, DcMotorSimple.Direction direction) {
        this.frontLeftName = name;
        this.frontLeftDirection = direction;
        return this;
    }
    public MecanumConfig backLeft(String name, DcMotorSimple.Direction direction) {
        this.backLeftName = name;
        this.backLeftDirection = direction;
        return this;
    }
    public MecanumConfig frontRight(String name, DcMotorSimple.Direction direction) {
        this.frontRightName = name;
        this.frontRightDirection = direction;
        return this;
    }
    public MecanumConfig backRight(String name, DcMotorSimple.Direction direction) {
        this.backRightName = name;
        this.backRightDirection = direction;
        return this;
    }
    
    public MecanumConfig maxForwardSpeed(double maxForwardSpeed) {
        this.maxForwardSpeed = maxForwardSpeed;
        return this;
    }
    public MecanumConfig maxStrafeSpeed(double maxStrafeSpeed) {
        this.maxStrafeSpeed = maxStrafeSpeed;
        return this;
    }
    
    @Override
    public Drivetrain build(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, this);
    }
}
