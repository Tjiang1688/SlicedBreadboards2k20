package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

/**
 * Created by e.xing on 2/10/2017.
 */
//@TeleOp(name = "LinearSMTele", group = "TeleOp")
@Disabled
public class LinearSMTele extends LinearOpMode {
    /***
     * GAME VARIABLES
     */
    private Robot1 robot;
    private ElapsedTime runtime = new ElapsedTime();

    private final double PERIODIC_INTERVAL = 10; //in milliseconds
    private double nextPeriodicTime;

    private double startLoadTime = 0;
    int launchCount = 0;

    private double leftStick = 0;
    private double rightStick = 0;

    private boolean prevLaunchAdjust = false;

    private boolean debugOn = true;

    /***
     * STATE MACHINE + EVENTS
     */
    //consider as stack
    Deque<STATE> states = new ArrayDeque();

    private enum STATE {
        //DRIVE,
        STOP,

        LAUNCH,
        LAUNCH_READY,
        LAUNCH_FIRE,
        LAUNCH_RESET,

        LOAD,
        LOAD_OPEN,
        LOAD_CLOSE,
    }

    public enum INPUT {
        GAMEPAD_BUTTON_BUMPER_RIGHT,
        GAMEPAD_STICK_LEFT,
        GAMEPAD_STICK_RIGHT,
        GAMEPAD_BUTTON_A,
        GAMEPAD_BUTTON_B,
        GAMEPAD_BUTTON_X,
        GAMEPAD_BUTTON_Y,
    }

    int count;

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
        //This loop will exit once the opmode is closed
        while (opModeIsActive()) {
            if (runtime.milliseconds() >= nextPeriodicTime) {
                nextPeriodicTime = runtime.milliseconds() + PERIODIC_INTERVAL;
                periodicLoop();
            }
            continuousLoop();

            count++;
            telemetry.addData("count", count);
            telemetry.update();
        }

        stopSB();
    }

    public void initSB() {
        robot = new Robot1();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);
    }

    public void startSB() {
        robot.pressBeaconSide("left");
        runtime.reset();
    }

    public void periodicLoop() {
        for (INPUT i : INPUT.values()) {
            processInput(i);
        }
    }

    public void continuousLoop() {
        STATE currState = states.peekLast();
        if (currState != null) {
            telemetry.addData("State:", currState +"");
            telemetry.update();
            stateMachine(currState);
        }
        stateDrive();

        telemetry.addData("curr stack", Arrays.toString(states.toArray()));
    }

    public void stopSB() {
        robot.drive.stop();
        robot.launcher.stop();
        robot.collectMotor.setPower(0);
    }

    public void stateDrive() {
        double leftPow = 0;
        double rightPow = 0;

        if (leftStick != 0) {
            leftPow = leftStick;
            leftPow = drivePowerScale(leftPow);
        }
        if (rightStick != 0) {
            rightPow = rightStick;
            rightPow = drivePowerScale(rightPow);
        }

        //normalizes motor power values
        double maxPow = Math.max(Math.abs(leftPow), Math.abs(rightPow));
        if (maxPow > 1.0)
        {
            leftPow /= maxPow;
            rightPow /= maxPow;
        }

        leftPow = truncate(leftPow);
        rightPow = truncate(rightPow);

        /*
        leftPow = scale(leftPow, drivePowerScale(-1), drivePowerScale(1), -1, 1);
        rightPow = scale(rightPow, drivePowerScale(-1), drivePowerScale(1), -1, 1);
        */

        robot.drive.powerDrive(leftPow, rightPow);
    }

    // takes in a val with valMin and valMax, scales to a number in range limMin to limMax
    public double scale(double val, double valMin, double valMax, double limMin, double limMax) {
        return ((limMax - limMin) * (val - valMin) / (valMax - valMin)) + valMin;
    }

    public double drivePowerScale(double n) {
        return Math.pow(n, 3) / 2;
    }

    public double truncate(double val) {
        if (val < 0) {
            val = 0.1 * Math.ceil(val * 10.0);
        } else {
            val = 0.1 * Math.floor(val * 10.0);
        }
        return val;
    }

    public void stateMachine(STATE currState) {
        switch (currState) {
            case STOP: {
                states.pop();
                robot.drive.stop();

                break;
            }

            case LAUNCH: {
                states.pop();

                if (launchCount == 0) { //then ready launch
                    robot.launcher.reset();

                    launchCount++;

                    states.push(STATE.LAUNCH_READY);
                    robot.launcher.ready();

                } else if (launchCount == 1 && !states.contains(STATE.LAUNCH_READY)) {
                    states.push(STATE.LAUNCH_FIRE);
                    robot.launcher.fire();

                    launchCount = 0;
                }

                break;
            }

            case LAUNCH_READY: {
                boolean moving = -robot.launchMotor.getCurrentPosition() > robot.launcher.READY_LAUNCH_POS;
                if (!moving) {
                    states.pop();
                    robot.launcher.stop();
                }

                break;
            }

            case LAUNCH_FIRE: {
                boolean moving = -robot.launchMotor.getCurrentPosition() > robot.launcher.FIRE_LAUNCH_POS;
                if (!moving) {
                    states.pop();
                    robot.launcher.stop();
                }

                break;
            }

            case LAUNCH_RESET: {
                states.pop();

                break;
            }

            case LOAD: {
                states.pop();

                if (startLoadTime == 0) {
                    startLoadTime = runtime.milliseconds();
                    states.push(STATE.LOAD_OPEN);
                    robot.launcher.openLoading();
                }

                break;
            }

            case LOAD_OPEN: {
                boolean done = runtime.milliseconds() >= (startLoadTime + robot.launcher.LOAD_TIME);

                telemetry.addData("load open", done);
                telemetry.update();

                if (done) {
                    states.pop();
                    states.push(STATE.LOAD_CLOSE);
                    robot.launcher.closeLoading();
                }

                break;
            }

            case LOAD_CLOSE: {
                states.pop();

                startLoadTime = 0;
                break;
            }
        }
    }

    public void processInput(INPUT currInput) {
        switch (currInput) {
            case GAMEPAD_BUTTON_BUMPER_RIGHT: {
                if (gamepad1.right_bumper == true) {
                    prevLaunchAdjust = true;
                    robot.launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.launchMotor.setPower(-0.05);
                } else if (prevLaunchAdjust == true) {
                    prevLaunchAdjust = false;
                    robot.launchMotor.setPower(0);
                }
            }

            case GAMEPAD_STICK_LEFT: {
                leftStick = -gamepad1.left_stick_y;
                break;
            }

            case GAMEPAD_STICK_RIGHT: {
                rightStick = -gamepad1.right_stick_y;
                break;
            }

            case GAMEPAD_BUTTON_A: {
                if (gamepad1.a == true && !states.contains(STATE.LOAD) && !states.contains(STATE.LOAD_CLOSE) && !states.contains(STATE.LOAD_OPEN)) {
                    states.push(STATE.LOAD);
                }
                break;
            }

            case GAMEPAD_BUTTON_B: {
                if (gamepad1.b == true && !states.contains(STATE.LAUNCH)) {
                    states.push(STATE.LAUNCH);
                }
                break;
            }

            case GAMEPAD_BUTTON_X: {
                if (gamepad1.x == true) {
                    robot.collectMotor.setPower(0.8);
                }
                break;
            }

            case GAMEPAD_BUTTON_Y: {
                if (gamepad1.y == true) {
                    robot.collectMotor.setPower(0);
                }
                break;
            }
        }
    }
}
