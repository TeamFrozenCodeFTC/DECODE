package org.firstinspires.ftc.blackice.core.hardware.localization.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.blackice.core.hardware.localization.Localizer;


/**
 * A global class that holds the odometry object and provides methods to update the odometry data.
 */
public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver odometry;
    private final DistanceUnit distanceUnit;

    private double heading;
    private double x;
    private double y;

    private double angularVelocity;
    private double xVelocity;
    private double yVelocity;

    public PinpointLocalizer(HardwareMap hardwareMap, PinpointConfig config)
        throws InterruptedException {
        this.distanceUnit = config.distanceUnit;
        
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, config.name);

        odometry.setOffsets(config.xPodOffset, config.yPodOffset);
        odometry.setEncoderResolution(config.podType);

        odometry.setEncoderDirections(
            config.xPodDirection,
            config.yPodDirection
        );
        odometry.resetPosAndIMU();
        
        Thread.sleep(300);
        
        update();
    }
    
    @Override
    public void update() {
        odometry.update();

        Pose2D position = odometry.getPosition();
        x = position.getX(distanceUnit);
        y = position.getY(distanceUnit);
        heading = position.getHeading(AngleUnit.RADIANS);

        Pose2D velocity = odometry.getVelocity();
        xVelocity = velocity.getX(distanceUnit);
        yVelocity = velocity.getY(distanceUnit);
        angularVelocity = velocity.getHeading(AngleUnit.RADIANS);
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getFieldVelocityX() {
        return xVelocity;
    }

    @Override
    public double getFieldVelocityY() {
        return yVelocity;
    }

    @Override
    public double getHeading() {
        return heading;
    }

    @Override
    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setPose(double x, double y, double heading) {
        odometry.setPosition(new Pose2D(
            distanceUnit,
            x,
            y,
            AngleUnit.DEGREES,
            heading
        ));
        update();
    }

    @Override
    public void reset() {
        odometry.recalibrateIMU();
    }
}
