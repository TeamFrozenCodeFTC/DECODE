//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.blackice.core.follower.Follower;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Paddles;
//import org.firstinspires.ftc.teamcode.subsystems.Ramp;
//import org.firstinspires.ftc.teamcode.subsystems.XXSpindexer;
//
//public class XXRobot {
//    public Intake intake;
//    public Flywheel2 launcher;
//    public XXSpindexer spindexer;
//    public Follower follower;
//    public Ramp intakeRamp;
//    public Paddles paddles;
//    public OpMode op;
//
//    // State Managers
//    public PrepareForIntakeState prepareForIntakeState = new PrepareForIntakeState();
//    public InitialState initialState = new InitialState();
//    public IntakeRunningState intakeRunningState = new IntakeRunningState();
//    public IntakeToSpindexerState intakeToSpindexerState = new IntakeToSpindexerState();
//
//
//    public enum StateEnum {
//        INITIAL,
//        IDLE,
//        PREPARE_INTAKE,      // can manually transition to this state
//        INTAKE_RUNNING,      // do not manually transition to this state (start at PREPARE_INTAKE)
//        INTAKE_TO_SPINDEXER, // do not manually transition to this state (start at PREPARE_INTAKE)
//        STOP_INTAKE,         // can manually transition to this state
//        ACTIVATE_LAUNCHER,
//        DEACTIVATE_LAUNCHER,
//        LAUNCH_MOTIF,
//        LAUNCH_SINGLE,
//        LAUNCH_ALL
//    }
//
//    public enum RampStateEnum {
//        INTAKE_TO_SPINDEXER,
//        INTAKE_TO_LAUNCHER,
//        DROP_TO_LAUNCHER
//    }
//
//    public enum PaddleStateEnum {
//        PADDLES_UP,
//        PADDLES_DOWN
//    }
//
//    // Robot State
//    public RampStateEnum rampState = RampStateEnum.INTAKE_TO_LAUNCHER;
//    public PaddleStateEnum paddleState = PaddleStateEnum.PADDLES_DOWN;
//    public StateEnum state = StateEnum.INITIAL;
//    public boolean intakeOn = false;
//
//    class Timer {
//        long milliseconds;
//        long start;
//
//        public void setAlarm(long milliseconds) {
//            this.start = System.currentTimeMillis();
//            this.milliseconds = milliseconds;
//        }
//
//        public boolean alarmTriggered() {
//            return (System.currentTimeMillis() - start) >= milliseconds;
//        }
//    }
//
//    class InitialState {
//        boolean inTransition = false;
//        Timer transitionTimer = new Timer();
//
//        public void checkTransition() {
//            // Always transition to IDLE state
//            if(!inTransition) {
//                state = StateEnum.IDLE;
//            }
//        }
//
//        public void update() {
//            if(!inTransition) {
//                inTransition = true;
//                transitionTimer.setAlarm(1500);
//
//                intakeRamp.intakeThrough();
//                rampState = RampStateEnum.INTAKE_TO_LAUNCHER;
//
//                paddles.open();
//                paddleState = PaddleStateEnum.PADDLES_DOWN;
//
//                spindexer.reset();
//
//                intake.stop();
//                intakeOn = false;
//
//            } else if (transitionTimer.alarmTriggered()) {
//                inTransition = false;
//            }
//        }
//    }
//
//    class PrepareForIntakeState {
//        boolean rampInTransition = false;
//        Timer rampTransitionTimer = new Timer();
//        boolean paddlesInTransition = false;
//        Timer paddleTransitionTimer = new Timer();
//
//        public void update() {
//
//            // Make sure the ramp is in the right position
//            if (rampState != RampStateEnum.INTAKE_TO_SPINDEXER) {
//                op.telemetry.addData("action", "moving ramp");
//                if(!rampInTransition) {
//                    rampInTransition = true;
//                    rampTransitionTimer.setAlarm(750);
//                    intakeRamp.uptake();
//                } else if(rampTransitionTimer.alarmTriggered()) {
//                    rampState = RampStateEnum.INTAKE_TO_SPINDEXER;
//                    rampInTransition = false;
//                }
//            }
//
//            // Make sure the intake motors are on
//            if(!intakeOn) {
//                intake.intake();
//                intakeOn = true;
//            }
//
//            // Make sure the paddles are down
//            if(paddleState != PaddleStateEnum.PADDLES_DOWN) {
//                if(!paddlesInTransition) {
//                    paddlesInTransition = true;
//                    paddleTransitionTimer.setAlarm(500);
//                    paddles.open();
//                } else if(paddleTransitionTimer.alarmTriggered()) {
//                    paddlesInTransition = false;
//                    paddleState = PaddleStateEnum.PADDLES_DOWN;
//                }
//            }
//        }
//
//        public void checkTransition() {
//            // If the ramp is in the right position and the intake motors are on
//            // and the paddles are down we are ready to intake an artifact
//            if(rampState == RampStateEnum.INTAKE_TO_SPINDEXER && paddleState == PaddleStateEnum.PADDLES_DOWN && intakeOn) {
//                state = StateEnum.INTAKE_RUNNING;
//            }
//        }
//    }
//
//    class IntakeRunningState {
//
//        public void update() {
//            // Let things keep running
//        }
//
//        public void checkTransition() {
//            // If an artifact is detected, switch to spindexer intake
//            Artifact detectedArtifact = spindexer.getDetectedArtifact();
//            if (detectedArtifact.isArtifact()) {
//                state = StateEnum.INTAKE_TO_SPINDEXER;
//            }
//        }
//    }
//
//    class IntakeToSpindexerState {
//        boolean paddlesInTransition = false;
//        Timer paddleTransitionTimer = new Timer();
//        boolean artifactLoaded = false;
//        boolean spindexInTransition = false;
//        Timer spindexRotationTimer = new Timer();
//
//        public void update () {
//            // Make sure intake motors are off
//            if(intakeOn) {
//                intake.stop();
//                intakeOn = false;
//            }
//
//            // Make sure the paddles (and artifact) are up
//            if(paddleState != PaddleStateEnum.PADDLES_UP) {
//                if(!paddlesInTransition) {
//                    paddlesInTransition = true;
//                    paddleTransitionTimer.setAlarm(750);
//                    paddles.close();
//                } else if (paddleTransitionTimer.alarmTriggered()) {
//                    paddleState = PaddleStateEnum.PADDLES_UP;
//                    paddlesInTransition = false;
//                }
//            } else if(!artifactLoaded) {
//                // Paddles are up, now we need to rotate spindexer
//                if(!spindexInTransition) {
//                    spindexInTransition = true;
//                    spindexRotationTimer.setAlarm(750);
//                    // rotate spindexer
//                } else if (spindexRotationTimer.alarmTriggered()) {
//                    spindexInTransition = false;
//                    artifactLoaded = true;
//                }
//            }
//        }
//
//        public void checkTransition() {
//            if(artifactLoaded) {
//                // check if spindexer if full
//                artifactLoaded = false;
//                state = StateEnum.PREPARE_INTAKE;
//            }
//        }
//    }
//
//    class LaunchMotifState {
//        boolean motifComplete = false;
//        XXPatterns patterns = new XXPatterns();
//        String actions;
//        int actionIndex;
//
//        public void setMotif(String motif) {
//            actions = patterns.get(spindexer.getLoadedPattern(), motif);
//            motifComplete = false;
//            actionIndex = 0;
//        }
//
//        public void update() {
//            if(motifComplete || actions == null) {
//                return;
//            }
//
//            if(actionIndex < actions.length()) {
//                char action = actions.charAt(actionIndex);
//                if(action == 'L') {
//
//                } else {
//
//                }
//                actionIndex++;
//            } else {
//                motifComplete = true;
//            }
//        }
//
//        public void checkTransition() {
//            if(actions == null) {
//                // rumble
//                state = StateEnum.IDLE;
//            } else if (motifComplete) {
//                state = StateEnum.IDLE;
//            }
//        }
//    }
//
//    public XXRobot(OpMode mode) {
//        this.op = mode;
//        follower = new Follower(op.hardwareMap);
//        intake = new Intake(op.hardwareMap);
//        spindexer = new XXSpindexer(op);
//        spindexer.setOffset(-0.0205);
//        intakeRamp = new Ramp(op.hardwareMap);
//        paddles = new Paddles(op.hardwareMap);
//        launcher = new Flywheel2(op.hardwareMap);
//    }
//
//    public void setState(StateEnum state) {
//        this.state = state;
//    }
//
//    public StateEnum getState() {
//        return this.state;
//    }
//
//    public void update() {
//        switch(state) {
//            case INITIAL:
//                initialState.update();
//                initialState.checkTransition();
//                break;
//            case IDLE:
//                break;
//            case PREPARE_INTAKE:
//                prepareForIntakeState.update();
//                prepareForIntakeState.checkTransition();
//                break;
//            case INTAKE_RUNNING:
//                intakeRunningState.update();
//                intakeRunningState.checkTransition();
//                break;
//            case INTAKE_TO_SPINDEXER:
//                intakeToSpindexerState.update();
//                intakeToSpindexerState.checkTransition();
//                break;
//            case STOP_INTAKE:
//                intake.stop();
//                intakeOn = false;
//                state = StateEnum.IDLE;
//                break;
//        }
//        intake.update();
//    }
//}
