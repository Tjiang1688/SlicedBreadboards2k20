package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.*;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

@Autonomous(name="StateMachine", group="vision opmode")  // @Autonomous(...) is the other common choice
public class StateMachine extends LinearVisionOpMode {
    /***
     * GAME VARIABLES
     */
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    private final double PERIODIC_INTERVAL = 100; //in milliseconds
    private double nextPeriodicTime;

    //test at tournament in field, configure variables
    static final double LIGHT_THRESHOLD = 0.5; //higher = more light reflected, whiter color
    static final double DISTANCE_BEACON_THRESHOLD = 25;

    static final int BEACON_ANALYSIS_CONFIDENCE = 50;

    private boolean debugOn = true;

    /***
     * STATE MACHINE + EVENTS
     */
    //consider as stacks
    Deque<STATE> states = new ArrayDeque();
    Deque<EVENT> events = new ArrayDeque();

    private enum STATE {
        IDLE,
        BEGIN,
        GRIPGLYPH,
        JEWELKNOCK,
        TOSCANPIC,
        SCANPIC,
        TOBOX,
        PLACEGLYPH,
        PARK, //included in placeglyph???





        FOLLOW_LINE,
        START_PATH,
        MOVE_PATH,
        STOP_PATH,
        ANALYZE_BEACON,
        LAUNCH_PARTICLE
    }

    private enum EVENT {
        LINE_DETECTED;
    }

    public void runOpMode() throws InterruptedException {
        initSB();

        if (debugOn) {
            //while (true) { telemetry.addData("Debug:", "init good"); }
        }

        //Wait for the match to begin, presses start button
        waitForStart();

        startSB();

        if (debugOn) {
            //while (true) { telemetry.addData("Debug:", "start good"); }
        }

        nextPeriodicTime = runtime.milliseconds();
        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once the opmode is closed
        while (opModeIsActive()) {

            if (runtime.milliseconds() >= nextPeriodicTime) {
                nextPeriodicTime = runtime.milliseconds() + PERIODIC_INTERVAL;
                periodicLoop();
            }
            continuousLoop();

            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }

        stopSB();
    }

    private void inputGameConfig() throws InterruptedException{
        telemetry.addData("Input team color", "Red (press b) or Blue (press x)");
        telemetry.update();
        while (!gamepad1.b && !gamepad1.x) {
            wait(10);
        }

        if (gamepad1.b == true) {
            robot.teamColor = TeamColor.red;
        } else {
            robot.teamColor = TeamColor.blue;
        }
        telemetry.addData("Chosen team color", robot.teamColor);
        telemetry.update();
        wait(1000);

        telemetry.addData("Input which side", "Left or right (use triggers)");
        telemetry.update();
        while (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5) {
            wait(10);
        }

        if (gamepad1.left_trigger >= 0.5) {
            robot.startPosition = StartPosition.left;
        } else {
            robot.startPosition = StartPosition.right;
        }
        telemetry.addData("Chosen start postion", robot.startPosition);
        telemetry.update();
        wait(1000);
    }

    private void initSB() throws InterruptedException {
        robot = new Robot2017(TeamColor.red, StartPosition.left);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);

        //inputGameConfig
    }


    private void startSB() {
        states.push(STATE.BEGIN);

        runtime.reset();
    }

    /**
     * A periodic loop that runs every PERIODIC_INTERAL milliseconds
     * Priority is higher than continuousLoop();
     */
    private void periodicLoop() throws InterruptedException {
        EVENT currEvent = events.peekLast();
        if (currEvent != null) {
            eventHandler(currEvent);
        }
    }

    private void eventHandler(EVENT currentEvent) {
        switch (currentEvent) {
            case LINE_DETECTED: {
                if (lineDetected()) {
                    events.pop(); //remove, event occured

                    states.pop(); //should pop the move_path_state used to get to line
                    states.push(STATE.FOLLOW_LINE);
                    states.push(STATE.STOP_PATH); //this state executes first b/c stack
                }
            }
        }
    }

    private boolean lineDetected() {
        telemetry.addData("Light", robot.ods.getLightDetected());
        return robot.ods.getLightDetected() >= LIGHT_THRESHOLD;
    }

    /**
     * A continuous loop that runs as fast as the system allows
     * Priority is lower than periodicLoop();
     */
    private void continuousLoop() throws InterruptedException {
        telemetry.clearAll();

        telemetry.addData("Events waiting for:", Arrays.toString(events.toArray()));
        telemetry.addData("States in stack:", Arrays.toString(states.toArray()));
        telemetry.update();

        STATE currState = states.peekLast();

        if (currState != null) {
            stateMachine(currState);
            sleep(3000);
        }
    }

    /**
     *
     * @param currentState
     * states.pop() removes the current state from the top of the stack.
     */
    public void stateMachine(STATE currentState) {
        switch (currentState) {
            case IDLE: {
                break;
            }

            case BEGIN: {
                states.pop();

                //go towards beacon
                robot.drive.queuePath(new PathSeg(48, 48, 0.4, runtime, 10000.0));
                events.push(EVENT.LINE_DETECTED);

                states.push(STATE.START_PATH);

                break;
            }

            case START_PATH: {
                states.pop();

                robot.drive.startPath();

                states.push(STATE.MOVE_PATH);

                break;
            }

            case MOVE_PATH: {
                boolean pathDone = robot.drive.pathDone();
                if (pathDone) { //path has ended
                    states.pop();
                    states.push(STATE.STOP_PATH);
                }

                break;
            }

            case STOP_PATH: {
                states.pop();

                robot.drive.stopCurrPath();

                break;
            }

            /**
             * https://ftc-tricks.com/proportional-line-follower/
             */
            case FOLLOW_LINE: {
                states.pop();

                followLine();

                states.push(STATE.ANALYZE_BEACON);

                break;
            }

            case ANALYZE_BEACON: {
                states.pop();

               telemetry.addData("done", "done");
                telemetry.update();
                break;
            }
        }
    }

    private boolean inFrontOfBeacon() {
        return robot.ultrasonic.getUltrasonicLevel() <= DISTANCE_BEACON_THRESHOLD;
    }

    private void followLine() {
        double leftPower = 0;
        double rightPower = 0;
        while ((!inFrontOfBeacon() || !beacon.getAnalysis().isBeaconFound()) && opModeIsActive()) {
            double correction = (LIGHT_THRESHOLD - robot.ods.getLightDetected());
            if (correction <= 0) {
                leftPower = 0.075d - correction;
                rightPower = 0.075d;
            } else {
                leftPower = 0.075d;
                rightPower = 0.075d + correction;
            }

            robot.drive.powerDrive(leftPower, rightPower);
        }
    }


    private void pressBeacon() {
        double leftPower = 0;
        double rightPower = 0;

        //move to beacon
        while (opModeIsActive()) {
            double correction = (0 - robot.ultrasonic.getUltrasonicLevel());
            if (correction <= 0) {
                leftPower = 0.075d - correction;
                rightPower = 0.075d;
            } else {
                leftPower = 0.075d;
                rightPower = 0.075d + correction;
            }

            robot.drive.powerDrive(leftPower, rightPower);

        }

        robot.drive.stop();

        //back up from beacon
        while (opModeIsActive()) {
            double correction = (robot.ultrasonic.getUltrasonicLevel() - DISTANCE_BEACON_THRESHOLD);
            if (correction <= 0) {
                leftPower = 0.075d - correction;
                rightPower = 0.075d;
            } else {
                leftPower = 0.075d;
                rightPower = 0.075d + correction;
            }

            robot.drive.powerDrive(leftPower, rightPower);

        }

        robot.drive.stop();
    }

    private void stopSB() {

    }
}

