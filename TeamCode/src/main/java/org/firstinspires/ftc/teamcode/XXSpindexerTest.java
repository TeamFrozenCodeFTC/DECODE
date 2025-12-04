package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.XXSpindexer;

@TeleOp
public class XXSpindexerTest extends LinearOpMode {
    public XXSpindexer spindexer;

    @Override
    public void runOpMode() throws InterruptedException {
        spindexer = new XXSpindexer(this);
        spindexer.setOffset(-0.0205); // .0651
        spindexer.reset();
        sleep(1000);

        waitForStart();
        boolean offsetLocked = false;
        while (opModeIsActive()) {
            if(gamepad1.dpad_left) {
                long delay = spindexer.rotateLeft(XXSpindexer.RotationAmount.THIRD);
                sleep(delay);
            } else if(gamepad1.dpad_up) {
                long delay = spindexer.rotateLeft(XXSpindexer.RotationAmount.SIXTH);
                sleep(delay);
            } else if (gamepad1.dpad_right) {
                long delay = spindexer.rotateRight(XXSpindexer.RotationAmount.THIRD);
                sleep(delay);
            } else if (gamepad1.dpad_down) {
                long delay = spindexer.rotateRight(XXSpindexer.RotationAmount.SIXTH);
                sleep(delay);
            }

            double offset = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.1;
            if(offsetLocked && offset == 0.0) {
                offsetLocked = false;
            }

            if(!offsetLocked) {
                spindexer.adjustServo(offset);
            }

            if(gamepad1.cross && !offsetLocked) {
                spindexer.lockAdjustment(offset);
                offsetLocked = true;
            }

//            telemetry.addData("servo position", spindexer.currentServoPosition);
//            telemetry.addData("slot", spindexer.currentSlotIndex);
//            telemetry.addData("offset", offset);
//            telemetry.addData("offset locked", offsetLocked);
//            telemetry.update();
        }
    }
}
