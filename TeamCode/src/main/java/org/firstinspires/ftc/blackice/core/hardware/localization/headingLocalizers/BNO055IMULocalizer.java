package org.firstinspires.ftc.blackice.core.hardware.localization.headingLocalizers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.blackice.core.hardware.localization.HeadingLocalizer;

/**
 * This is only to allow simple robot with an IMU to use field-centric driving.
 */
@Deprecated // IMU or BNO055IMU?
public class BNO055IMULocalizer implements HeadingLocalizer {
    private final BNO055IMU imu;
    private double headingOffset = 0;

    public BNO055IMULocalizer(HardwareMap hardwareMap, DistanceUnit distanceUnit) {
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        this.imu.initialize(parameters);
    }
    
    @Override
    public void update() {
    
    }
    
    @Override
    public double getHeading() {
        return AngleUnit.RADIANS.normalize(
            this.imu.getAngularOrientation().firstAngle - headingOffset
        );
    }
    
    @Override
    public void reset() {
        this.headingOffset = 0;
    }
    
    @Override
    public void setPose(double x, double y, double heading) {
        this.headingOffset = this.imu.getAngularOrientation().firstAngle
            - Math.toRadians(heading);
    }
}
