package org.firstinspires.ftc.teamcode.miniblackice.core;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.miniblackice.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.miniblackice.drivetrain.mecanum.MecanumConfig;
import org.firstinspires.ftc.teamcode.miniblackice.localizers.LocalizerConfig;
import org.firstinspires.ftc.teamcode.miniblackice.localizers.pinpoint.PinpointConfig;

public class FollowerConstants {
    public static LocalizerConfig localizerConfig = new PinpointConfig()
        .name("odo")
        .podDirection(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .podOffset(84, -168);
    
    public static DrivetrainConfig drivetrainConfig = new MecanumConfig()
        .frontLeft("frontLeft", DcMotorSimple.Direction.REVERSE)
        .backLeft("backLeft", DcMotorSimple.Direction.FORWARD)
        .frontRight("frontRight", DcMotorSimple.Direction.REVERSE)
        .backRight("backRight", DcMotorSimple.Direction.FORWARD)
        .maxForwardSpeed(60)
        .maxStrafeSpeed(45);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
//            new PDController(3, 0.15),
            new PDController(2, 0.15),
//            new PredictiveBrakingController(0.3, 0.0881, 0.00117),
            new PredictiveBrakingController(0.2, 0.0881, 0.00117),
            drivetrainConfig,
            localizerConfig,
            hardwareMap,
            new PoseTolerance(0.5, 3),
            new MotionTolerance(0.25, 5)
        );
    }
}
