package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Testrobot extends OpMode {
    DcMotor backleft;
    DcMotor backright;
    DcMotor frontleft;
    DcMotor frontright;
    @Override
    public void init() {
        backleft = hardwareMap.get(DcMotor.class, "bl");
        backright = hardwareMap.get(DcMotor.class, "br");
        frontleft = hardwareMap.get(DcMotor.class, "fl");
        frontright = hardwareMap.get(DcMotor.class, "fr");
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        backleft.setPower(gamepad1.left_stick_y);
        frontleft.setPower(gamepad1.left_stick_y);
        backright.setPower(gamepad1.right_stick_y);
        frontright.setPower(gamepad1.right_stick_y);
    }
}
