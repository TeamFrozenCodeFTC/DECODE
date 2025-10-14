package org.firstinspires.ftc.blackice;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.blackice.core.hardware.localization.localizers.GoBildaPinpointDriver;
import org.firstinspires.ftc.blackice.core.hardware.localization.localizers.LocalizerConfig;
import org.firstinspires.ftc.blackice.core.hardware.localization.localizers.PinpointConfig;
import org.firstinspires.ftc.blackice.core.hardware.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.blackice.core.control.QuadraticDampedPIDController;
import org.firstinspires.ftc.blackice.core.follower.FollowerConfig;
import org.firstinspires.ftc.blackice.core.control.PIDFController;
import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.core.hardware.drivetrain.MecanumConfig;

/**
 * Set default configuration for all OpModes using the Follower.
 */
public final class FollowerConstants {
    public static LocalizerConfig localizerConfig = new PinpointConfig()
        .distanceUnit(DistanceUnit.INCH)
        .podDirection(
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.REVERSED)
        .podOffset(-36, 0);
    
    public static DrivetrainConfig drivetrainConfig = new MecanumConfig()
        .frontLeft("frontLeft", DcMotorSimple.Direction.REVERSE)
        .backLeft("backLeft", DcMotorSimple.Direction.REVERSE)
        .frontRight("frontRight", DcMotorSimple.Direction.REVERSE)
        .backRight("backRight", DcMotorSimple.Direction.FORWARD)
        .maxForwardSpeed(65.37) // Tuned for 312 rpm
        .maxStrafeSpeed(46.22); // * 1/sqrt(2) for diagonal
    
    public static PathBehavior defaultPathBehavior = new PathBehavior()
        .continueMomentumAtEnd()
        .setStuckTimeoutSeconds(3.0)
        .setPauseTimeoutSeconds(5.0)
        .setTimeoutSeconds(8.0);
    
    public static FollowerConfig defaultFollowerConfig = new FollowerConfig()
        .localizerConfig(localizerConfig)
        .drivetrainConfig(drivetrainConfig)
        .defaultPathBehavior(defaultPathBehavior)
        .headingPID(new PIDFController(2, 0, 0.1, 0.01, 0))
        .positionalPID(new QuadraticDampedPIDController(0.5, 0.07, 0.001, 0.015))
        .driveVelocityController(new PIDFController(0.01, 0, 0, 0.04, 0.0159))
        .naturalDeceleration(40) // only needed for following velocity profiles.
        .maxReversalBrakingPower(0.2) // has crashed once at -0.3 but is a lot easier
        // to manage when crashed
        .tunedVoltage(12.0)
        .centripetalFeedforward(0.005) // only needed for following curves
        .stopIfVoltageBelow(7);
}

// ! https://visualizer.pedropathing.com

//public static VelocityToStoppingDistanceVectorModel BRAKING_DISPLACEMENT =
//    new VelocityToStoppingDistanceVectorModel(
//        //Drift, braking force
//        new QuadraticLinearBrakingModel(0.00112, 0.07316),
//        new QuadraticLinearBrakingModel(0.00165, 0.05054)
//    );

//
//Heading 0 is the front side of the robot facing away from the starting wall
//positive X-axis is forward for the robot with 0 heading (facing the center of the field)
//positive Y-axis is strafing the robot to the left from 0 heading
//
//This is the axis with the robot's HEADING AT 0 DEGREES: The robot is touching the wall
//    ```
//    ^ +Y Axis (Robot Left Strafing)
//│
//    │ ┌──────────┐
//    │ │          │
//    │ │          │--> FRONT OF ROBOT (FORWARD)
//│ │          │
//    │ └──────────┘
//    └──────────────> +X Axis (Robot Forward)
//```
