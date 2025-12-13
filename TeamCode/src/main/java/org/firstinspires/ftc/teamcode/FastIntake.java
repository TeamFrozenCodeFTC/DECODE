package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class FastIntake extends TeleOps {
    Robot robot;
    
    State state = State.IDLE;
    int numberOfArtifacts = 0;
    boolean artifactBeingPushedUp = false;
    
    @Override
    public void loop() {
        switch (state) {
            case FAST_INTAKE: // Loads 1st with paddles and 2nd two in the intake
                robot.launcher.stop();
                robot.intakeRamp.uptake();
                robot.intake.intake();
                
                Artifact detectedArtifact = robot.spindexer.getDetectedArtifact();
                boolean artifactPresent = detectedArtifact.isArtifact();
                
                if (artifactPresent && numberOfArtifacts == 0) {
                    robot.paddles.close();
                    numberOfArtifacts++;
                }
                break;
            case FAST_SPINDEXER_LOAD: // loads 1st into spindexer from push from //
                // second, and third pushes second, and third is pushed by paddles
                robot.launcher.stop();
                robot.intakeRamp.uptake();
                robot.intake.intake();
                
                // purple -> intaked = purple, artifactBeingPushedUp = false
                // none -> artifactBeingPushedUp = true
                // purple -> spindexer rotates, artifactBeingPushedUp = false
                // none -> artifactBeingPushedUp = true
                // purple -> spindexer rotates, artifactBeingPushedUp = false, state =
                // IDLE
                
                Artifact detectedArtifact2 = robot.spindexer.getDetectedArtifact();
                boolean newArtifactPresent =
                    detectedArtifact2.isArtifact() && !artifactBeingPushedUp;
                if (newArtifactPresent) {
                    robot.intakedArtifact = detectedArtifact2;
                    if (robot.spindexer.getNumberOfArtifacts() == 2) {
                        robot.paddles.close();
                    }
                }
                // Intaked artifact no longer detected, meaning it is being pushed up into
                // the spindexer.
                if (robot.intakedArtifact.isArtifact() && detectedArtifact2.isNone()) {
                    artifactBeingPushedUp = true;
                }
                
                // When an artifact has been fully pushed up into the spindexer from
                // detecting another artifact underneath.
                boolean artifactIsInSpindexer =
                    artifactBeingPushedUp && detectedArtifact2.isArtifact();
                
                // Rotates spindexer to intake the artifact that was pushed up.
                if (artifactIsInSpindexer) {
                    robot.spindexer.intakeArtifact(robot.intakedArtifact);
                    robot.intakedArtifact = Artifact.NONE;
                    artifactBeingPushedUp = false;
                    if (robot.spindexer.getNumberOfArtifacts() == 3) {
                        state = State.IDLE;
                    }
                }
                break;
            case FIRE: // Fires 3 artifacts (1 stored in spindexer, 2 in intake)
                robot.intake.stop();
                robot.launcher.setRpmFromDistance(75);
                
                if (robot.launcher.artifactLaunched()) {
                    numberOfArtifacts -= 1;
                }
                
                if (!robot.launcher.isUpToSpeed()) {
                    break;
                }
                
                switch (numberOfArtifacts) {
                    case 3:
                        robot.intakeRamp.outtake();
                        robot.paddles.disable();
                        break;
                    case 2:
                        robot.intake.intake();
                        robot.intakeRamp.intakeThrough();
                        break;
                    case 1:
                        robot.paddles.open();
                        robot.intakeRamp.outtake();
                        break;
                    case 0:
                        state = State.IDLE;
                }
                
                break;
            case SALVO:
                if (robot.launcher.artifactLaunched()) {
                    robot.spindexer.slots[robot.spindexer.getNumberOfArtifacts()] =
                        Artifact.NONE;
                }
                
                if (robot.launcher.isUpToSpeed()) {
                    robot.spindexer.rotateToSlot(0);
                    
                    if (robot.spindexer.getNumberOfArtifacts() == 0) {
                        robot.setState(Robot.State.IDLE);
                        break;
                    }
                }
            case IDLE:
                robot.launcher.stop();
                robot.intake.stop();
                robot.intakeRamp.uptake();
                break;
        }
        
        robot.follower.fieldCentricTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x
        );
        
        if (gamepad1.rightBumperWasPressed()) {
            numberOfArtifacts = 3;
            state = State.FIRE;
        }
        if (gamepad1.triangleWasPressed()) {
            numberOfArtifacts = 0;
            state = State.FAST_INTAKE;
        }
        if (gamepad1.leftBumperWasPressed()) {
            state = State.IDLE;
        }
        if (gamepad1.right_trigger == 1) {
            state = State.SALVO;
        }
        if (gamepad1.left_trigger == 1) {
            state = State.FAST_SPINDEXER_LOAD;
        }
        
        
        super.loop();
    }
    
    enum State {
        FAST_INTAKE,
        FIRE,
        IDLE,
        FAST_SPINDEXER_LOAD,
        SALVO
    }
}
