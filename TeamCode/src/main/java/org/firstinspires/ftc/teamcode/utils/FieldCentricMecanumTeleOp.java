package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric Mecanum", group = "Drive")
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    
    private DcMotorEx fl, fr, bl, br;
    private IMU imu;
    
    
    
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        ));
        
        telemetry.addLine("Ready");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            
            double heading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
            
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            
            double rotX = x * cos - y * sin;
            double rotY = x * sin + y * cos;
            
            double denominator = Math.max(
                Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1
            );
            
            double boost = 0;
            if (gamepad1.right_bumper) {
                boost = 1;
            }
            
            double flPower = (rotY + rotX + rx) / denominator + boost;
            double blPower = (rotY - rotX + rx) / denominator + boost;
            double frPower = (rotY - rotX - rx) / denominator + boost;
            double brPower = (rotY + rotX - rx) / denominator + boost;
            
            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);
            
            if (gamepad1.options) {
                imu.resetYaw();
            }
            
            telemetry.addData("Heading (deg)",
                              imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
