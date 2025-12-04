//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp
//public class XXRobotTest extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        XXRobot robot = new XXRobot(this);
//        waitForStart();
//
//        // Initialize and transition to IDLE
//        robot.setState(XXRobot.StateEnum.INITIAL);
//        while(opModeIsActive()) {
//            robot.update();
//            telemetry.addData("initalizing - state", robot.getState().name());
//            telemetry.update();
//            if(robot.getState() == XXRobot.StateEnum.IDLE) {
//                break;
//            }
//        }
//
//        // Prepare intake and transition to INTAKE_RUNNING
//        robot.setState(XXRobot.StateEnum.PREPARE_INTAKE);
//        long start = System.currentTimeMillis();
//        while(opModeIsActive()) {
//            robot.update();
//            telemetry.addData("preparing - state", robot.getState().name());
//            telemetry.update();
//            if(robot.getState() == XXRobot.StateEnum.INTAKE_RUNNING && System.currentTimeMillis() - start > 2000) {
//                break;
//            }
//        }
//
//
//        // Stop intake and transition to IDLE
//        robot.setState(XXRobot.StateEnum.STOP_INTAKE);
//        start = System.currentTimeMillis();
//        while(opModeIsActive()) {
//            robot.update();
//            telemetry.addData("stopping - state", robot.getState().name());
//            telemetry.update();
//            if(robot.getState() == XXRobot.StateEnum.IDLE && System.currentTimeMillis() - start > 2000) {
//                break;
//            }
//        }
//    }
//}
