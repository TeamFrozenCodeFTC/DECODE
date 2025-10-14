package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;

@TeleOp
public class Test extends OpMode {
    private Menu menu;
    
    private static <T extends HardwareDevice> Map<String, T> getSortedHardware(HardwareMap hw,
                                                                               Class<T> clazz) {
        Map<String, T> map = new LinkedHashMap<>();
        hw.getAll(clazz)
            .stream()
            .sorted(Comparator.comparing(dev -> hw.getNamesOf(dev).iterator().next()))
            .forEach(dev -> {
                String name = hw.getNamesOf(dev).iterator().next();
                map.put(name, dev);
            });
        return map;
    }
    
    @Override
    public void init() {
        menu = new Menu(gamepad1);
        Map<String, ServoImplEx> servos = getSortedHardware(hardwareMap, ServoImplEx.class);
        Map<String, DcMotorEx> motors = getSortedHardware(hardwareMap, DcMotorEx.class);
        
        // Servo Submenu
        Menu servoMenu = new Menu(gamepad1);
        for (Map.Entry<String, ServoImplEx> entry : servos.entrySet()) {
            String name = entry.getKey();
            ServoImplEx servo = entry.getValue();
            final double[] servoPos = {servo.getPosition()};
            servoMenu.addOption(name, () -> {
                servo.setPwmEnable();
                double delta = -gamepad1.right_stick_y * 0.005;
                servoPos[0] = clamp(servoPos[0] + delta, 0, 1);
                servo.setPosition(servoPos[0]);
                telemetry.addData("servoPos", "%.3f", servoPos[0]);
                telemetry.addData("servoAngle", "%.2f", servoPos[0] * 360);
            }, servo::setPwmDisable);
        }
        
        // Motor Submenu
        Menu motorMenu = new Menu(gamepad1);
        for (Map.Entry<String, DcMotorEx> entry : motors.entrySet()) {
            String name = entry.getKey();
            DcMotorEx motor = entry.getValue();
            motorMenu.addOption(name, () -> {
                motor.setMotorEnable();
                double power = -gamepad1.right_stick_y;
                motor.setPower(power);
                telemetry.addData("motorPower", "%.2f", power);
            }, motor::setMotorDisable);
        }
        
        // Main Menu
        menu.addSubmenu("Test Servos", servoMenu);
        menu.addSubmenu("Test Motors", motorMenu);
    }
    
    @Override
    public void loop() {
        menu.update();
        telemetry.addLine(menu.getDisplay());
        telemetry.update();
    }
    
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
