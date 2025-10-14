package org.firstinspires.ftc.blackice.core.hardware.localization.headingLocalizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.blackice.core.hardware.localization.HeadingLocalizer;

/**
 * This is only to allow simple robot with an IMU to use field-centric driving.
 */
public class IMULocalizer implements HeadingLocalizer {
    private final IMU imu;

    private double headingOffset = 0;

    public IMULocalizer(HardwareMap hardwareMap, DistanceUnit distanceUnit) {
        this.imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
            logoDirection, usbDirection
        );
        
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void update() {

    }
    
    private double getRawHeading() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public double getHeading() {
        return AngleUnit.RADIANS.normalize(getRawHeading() - headingOffset);
    }

    @Override
    public void reset() {
        this.headingOffset = 0;
    }
    
    @Override
    public void setPose(double x, double y, double heading) {
        this.headingOffset = getRawHeading() - Math.toRadians(heading);
    }
}
