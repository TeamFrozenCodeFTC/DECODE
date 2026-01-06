package org.firstinspires.ftc.teamcode.blackice.localizers.pinpoint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackice.localizers.Localizer;

class PinpointLocalizer extends Localizer {
    private final GoBildaPinpointDriver
        odometry;
    private final DistanceUnit distanceUnit;
    
    private Pose position;
    private Vector velocity; // field relative velocity
    private double angularVelocity; // radians per second

    public PinpointLocalizer(HardwareMap hardwareMap, PinpointConfig config) {
        this.distanceUnit = config.distanceUnit;
        
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, config.name);
        
        odometry.setOffsets(config.xPodOffset, config.yPodOffset, DistanceUnit.MM);
        odometry.setEncoderResolution(config.podType);

        odometry.setEncoderDirections(
            config.xPodDirection,
            config.yPodDirection
        );
        odometry.resetPosAndIMU();

        update(-1);
    }
    
    @Override
    public void update(double deltaTime) {
        odometry.update();
        
        position = new Pose(odometry.getPosX(distanceUnit),
                            odometry.getPosY(distanceUnit),
                            odometry.getHeading(AngleUnit.DEGREES)); // gets converted
        // to Radians internally

        velocity = new Vector(
            odometry.getVelX(distanceUnit),
            odometry.getVelY(distanceUnit)
        );
        
        angularVelocity = odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public void setCurrentPose(double x, double y, double heading) {
        odometry.setPosition(new Pose2D(
            distanceUnit,
            x,
            y,
            AngleUnit.RADIANS,
            heading
        ));
    }
    
    @Override
    public Pose getPose() {
        return position;
    }
    
    @Override
    public Vector getVelocity() {
        return velocity;
    }
    
    @Override
    public double getAngularVelocity() {
        return angularVelocity;
    }
    
    @Override
    public void reset() {
        odometry.recalibrateIMU();
    }
}
