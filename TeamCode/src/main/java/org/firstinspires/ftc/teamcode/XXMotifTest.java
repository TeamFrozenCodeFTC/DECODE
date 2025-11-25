package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.XXSpindexer;

@TeleOp
public class XXMotifTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        XXPatterns patterns = new XXPatterns();
        XXSpindexer spindexer = new XXSpindexer(this);
        spindexer.setOffset(-0.0205);
        spindexer.reset();

        waitForStart();

        // Simulate loading P-P-G
        spindexer.pushArtifact(Artifact.GREEN); // slot 2
        sleep(spindexer.rotateRight(XXSpindexer.RotationAmount.THIRD) + 250);
        spindexer.pushArtifact(Artifact.PURPLE); // slot 0
        sleep(spindexer.rotateRight(XXSpindexer.RotationAmount.THIRD) + 250);
        spindexer.pushArtifact(Artifact.PURPLE); // slot 1
        sleep(spindexer.rotateRight(XXSpindexer.RotationAmount.SIXTH) + 250);
        String spinPattern = spindexer.getLoadedPattern();

        telemetry.addData("loaded pattern", spinPattern);

        // Pick an output pattern and get the corresponding action sequence
        // String actions = patterns.get(spinPattern, "PPG");
        String actions = patterns.get(spinPattern, "PGP");
        //String actions = patterns.get(spinPattern, "GPP");

        telemetry.addData("action pattern", actions);
       telemetry.update();


       // Work through the action sequence
        long delay = 0;
        for(int i = 0; i < actions.length(); i++) {
           char a = actions.charAt(i);
           telemetry.addData("loaded pattern", spinPattern);
           telemetry.addData("action pattern", actions);
           telemetry.addData("index", i);
           telemetry.addData("action", a);
           if(a == 'L') {
               if(spindexer.isFullyRotated()) {
                   delay = spindexer.rotateLeft(XXSpindexer.RotationAmount.THIRD);
               } else {
                   delay = spindexer.rotateLeft(XXSpindexer.RotationAmount.SIXTH);
               }
           } else if (a == 'R') {
               if(spindexer.isFullyRotated()) {
                   delay = spindexer.rotateRight(XXSpindexer.RotationAmount.THIRD);
               } else {
                   delay = spindexer.rotateRight(XXSpindexer.RotationAmount.SIXTH);
               }
           }
           telemetry.update();
           sleep(delay);
       }
        sleep(2000);
    }
}
