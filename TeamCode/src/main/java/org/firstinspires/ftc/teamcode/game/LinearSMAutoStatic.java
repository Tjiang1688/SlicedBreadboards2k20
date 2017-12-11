package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.PathSeg;
import org.firstinspires.ftc.teamcode.game.robot.Robot1;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

/**
 * Created by e.xing on 2/9/2017.
 */
//@Autonomous(name="LinearSMAutoStatic", group="auto")  // @Autonomous(...) is the other common choice
@Disabled
public class LinearSMAutoStatic extends LinearVisionOpMode {
    /***
     * VISION VARIABLES
     */

    int frameCount = 0;

    private Cameras BEACON_CAMERA = Cameras.PRIMARY;
    private Beacon.AnalysisMethod BEACON_ANALYSIS_METHOD = Beacon.AnalysisMethod.REALTIME;
    private ScreenOrientation SCREEN_ORIENTATION = ScreenOrientation.LANDSCAPE;

    /***
     * GAME VARIABLES
     */
    private Robot1 robot;
    private ElapsedTime runtime = new ElapsedTime();

    private final double PERIODIC_INTERVAL = 100; //in milliseconds
    private double nextPeriodicTime;

    //test at tournament in field, configure variables
    static final double EDGE_LINE_THRESHOLD = 0.45; //higher = more light reflected, whiter color
    static final double MID_LINE_THRESHOLD = 0.91;

    static final double DISTANCE_BEACON_THRESHOLD = 35;
    static final double HIT_BEACON_THRESHOLD = 8;

    static final double BEACON_ANALYSIS_CONFIDENCE = 0.75;

    private boolean debugOn = true;

    public int beaconCount = 1;

    /***
     * STATE MACHINE + EVENTS
     */
    //consider as stacks
    Deque<STATE> states = new ArrayDeque();
    Deque<EVENT> events = new ArrayDeque();

    private enum STATE {
        IDLE,
        LAUNCH,
        BEGIN,
        FOLLOW_LINE,
        START_PATH,
        MOVE_PATH,
        STOP_PATH,
        ANALYZE_BEACON,
        LAUNCH_PARTICLE,
        OPEN_LOADING,
        CLOSE_LOADING,
        PARK;
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
            checkLatestFrame();

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
        while (gamepad1.b != true && gamepad1.x != true) {
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

        /*        telemetry.addData("Input which side", "Left or right (use triggers)");
        telemetry.update();
        while (!(gamepad1.left_trigger >= 0.5) && !(gamepad1.right_trigger >= 0.5)) {
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
        */
    }

    private void initSB() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();
        initVision();

        robot = new Robot1(TeamColor.blue, StartPosition.left);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);

        //inputGameConfig();
    }

    private void initVision() {
        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(BEACON_CAMERA);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set beacon analysis method
         * FAST
         * - analyzes frames at 5FPS
         * - looks for primary colors in image uses less battery,
         * - faster, uses less battery
         * - most accurate at less than 1.5feet away
         * - Fails if something red or blue is larger than beacon
         *
         * COMPLEX
         * - slower, but provides detection confidence
         * - most accurate at 1-4feet
         * - better at removing background noise
         * - uses statistical analysis to determine beacon
         * - analyzes frames at 2-4FPS
         *
         * REALTIME
         * - retrieves frames, analyzes at fast as possible (up to 15FPS)
         */
        beacon.setAnalysisMethod(BEACON_ANALYSIS_METHOD);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));


        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(SCREEN_ORIENTATION);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

    }

    private void startSB() {
        states.push(STATE.LAUNCH);
        //states.push(STATE.BEGIN);

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
        telemetry.addData("waiting for", "line");
        telemetry.addData("Light", robot.ods.getLightDetected());
        telemetry.update();
        return robot.ods.getLightDetected() > EDGE_LINE_THRESHOLD;
    }

    /**
     * A continuous loop that runs as fast as the system allows
     * Priority is lower than periodicLoop();
     */
    private void continuousLoop() throws InterruptedException {
        telemetry.clearAll();

        logVisionData();

        telemetry.addData("Events waiting for:", Arrays.toString(events.toArray()));
        telemetry.addData("States in stack:", Arrays.toString(states.toArray()));
        telemetry.update();

        STATE currState = states.peekLast();

        if (currState != null) {
            telemetry.addData("curr state", currState);
            telemetry.update();
            sleep(700);
            stateMachine(currState);
        }
    }

    /**
     *
     * @param currentState
     * states.pop() removes the current state from the top of the stack.
     */
    public void stateMachine(STATE currentState) throws InterruptedException {
        switch (currentState) {
            case IDLE: {
                break;
            }

            case LAUNCH: {
                states.pop();
                launchBalls();
                states.push(STATE.BEGIN);
            }

            case BEGIN: {
                states.pop();

                //rotate
                robot.drive.queuePath(new PathSeg(14 * 1.08, -14 * 1.08, runtime));
                robot.drive.startPath();

                while (!robot.drive.pathDone() && opModeIsActive()) {
                    sleep(10);
                }
                robot.drive.stopCurrPath();

                sleep(10);

                robot.drive.queuePath(new PathSeg(12 * 7.2, 12 * 7.2, runtime));
                robot.drive.startPath();

                while (!robot.drive.pathDone() && opModeIsActive()) {
                    sleep(10);
                }
                robot.drive.stopCurrPath();



                while (!lineDetected() && opModeIsActive()){
                    robot.drive.powerDrive(-0.2, 0.2);
                }
                robot.drive.powerDrive(0, 0);

                telemetry.addData("line", "found");
                telemetry.update();

                states.push(STATE.FOLLOW_LINE);

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

                if (beaconCount == 2) {
                    if (robot.teamColor.equals(TeamColor.blue)) {
                        robot.drive.queuePath(new PathSeg(14, -14, runtime));

                    } else {
                        robot.drive.queuePath(new PathSeg(-14, 14, runtime));
                    }

                    robot.drive.startPath();

                    while (!robot.drive.pathDone() && opModeIsActive()) {
                        sleep(10);
                    }
                    robot.drive.stopCurrPath();

                    telemetry.addData("rotate", "done");
                    telemetry.update();
                    sleep(1000);
                }

                followLine();

                states.push(STATE.ANALYZE_BEACON);

                break;
            }

            case ANALYZE_BEACON: {
                states.pop();

                analyzeBeacon();

                beaconCount++;
                telemetry.addData("beaconcount", beaconCount);
                telemetry.update();
                sleep(500);

                if (beaconCount == 2) {
                    if (robot.teamColor.equals(TeamColor.blue)) {
                        robot.drive.queuePath(new PathSeg(-14 * 0.85, 14 * 0.85, runtime));
                    } else {
                        robot.drive.queuePath(new PathSeg(14 * 0.85, -14 * 0.85, runtime));
                    }

                    robot.drive.startPath();

                    while (!robot.drive.pathDone() && opModeIsActive()) {
                        sleep(10);
                    }
                    robot.drive.stopCurrPath();

                    telemetry.addData("rotate", "done");
                    telemetry.update();
                    sleep(1000);

                    robot.drive.queuePath(new PathSeg(12 * 1.8, 12 * 1.8, runtime));
                    events.push(EVENT.LINE_DETECTED);
                    states.push(STATE.START_PATH);

                } else {
                    states.push(STATE.PARK);
                }

                break;
            }

            case PARK: {
                if (robot.teamColor.equals(TeamColor.blue)) {
                    robot.drive.queuePath(new PathSeg(-14 * 0.1, 14 * 0.1, runtime));
                } else {
                    robot.drive.queuePath(new PathSeg(14 * 0.1, -14 * 0.1, runtime));
                }
                robot.drive.startPath();

                while (!robot.drive.pathDone() && opModeIsActive()) {
                    sleep(10);
                }
                robot.drive.stopCurrPath();

                telemetry.addData("rotate", "done");
                telemetry.update();
                sleep(1000);

                robot.drive.queuePath(new PathSeg(-48, -48, runtime));
                states.push(STATE.START_PATH);
            }

        }
    }

    private void launchBalls() throws InterruptedException {
        robot.launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.launchMotor.setTargetPosition(robot.launcher.READY_LAUNCH_POS);
        robot.launchMotor.setPower(robot.launcher.LAUNCH_POWER);
        while (robot.launchMotor.isBusy() && opModeIsActive() && -robot.launchMotor.getCurrentPosition() > robot.launcher.READY_LAUNCH_POS && opModeIsActive()) {
            telemetry.addData("ready launch, curr pos", robot.launchMotor.getCurrentPosition());
            telemetry.addData("desired pos", robot.launcher.READY_LAUNCH_POS);
            telemetry.update();
        }
        robot.launchMotor.setPower(0);

        sleep(10);

        robot.launchMotor.setTargetPosition(robot.launcher.FIRE_LAUNCH_POS);
        robot.launchMotor.setPower(robot.launcher.LAUNCH_POWER);
        while (robot.launchMotor.isBusy() && opModeIsActive() && -robot.launchMotor.getCurrentPosition() > robot.launcher.FIRE_LAUNCH_POS && opModeIsActive()) {
            telemetry.addData("fire launch pos", robot.launchMotor.getCurrentPosition());
            telemetry.addData("desired pos", robot.launcher.FIRE_LAUNCH_POS);
            telemetry.update();
        }
        robot.launchMotor.setPower(0);

        robot.launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.launchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(10);

        robot.launchMotor.setTargetPosition(robot.launcher.READY_LAUNCH_POS);
        robot.launchMotor.setPower(robot.launcher.LAUNCH_POWER);
        while (robot.launchMotor.isBusy() && opModeIsActive() && -robot.launchMotor.getCurrentPosition() > robot.launcher.READY_LAUNCH_POS && opModeIsActive()) {
            telemetry.addData("ready launch, curr pos", robot.launchMotor.getCurrentPosition());
            telemetry.addData("desired pos", robot.launcher.READY_LAUNCH_POS);
            telemetry.update();
        }
        robot.launchMotor.setPower(0);

        sleep(100);

        robot.launcher.openLoading();
        sleep(robot.launcher.LOAD_TIME + 500);

        robot.launchMotor.setTargetPosition(robot.launcher.FIRE_LAUNCH_POS);
        robot.launchMotor.setPower(robot.launcher.LAUNCH_POWER);
        while (robot.launchMotor.isBusy() && opModeIsActive() && -robot.launchMotor.getCurrentPosition() > robot.launcher.FIRE_LAUNCH_POS && opModeIsActive()) {
            telemetry.addData("fire launch pos", robot.launchMotor.getCurrentPosition());
            telemetry.addData("desired pos", robot.launcher.FIRE_LAUNCH_POS);
            telemetry.update();
        }
        robot.launchMotor.setPower(0);
    }

    private boolean inFrontOfBeacon() throws InterruptedException {
        telemetry.addData("ultrasonic status", robot.ultrasonic.getUltrasonicLevel());
        telemetry.update();
        sleep(500);
        return robot.ultrasonic.getUltrasonicLevel() <= DISTANCE_BEACON_THRESHOLD;
    }

    private void followLine() throws InterruptedException {
        telemetry.addData("State", "follow line");
        telemetry.update();
        sleep(500);

        double leftPower = 0;
        double rightPower = 0;

        while (!inFrontOfBeacon() && opModeIsActive()) {
            double correction = (MID_LINE_THRESHOLD - robot.ods.getLightDetected()) / 5;
            if (correction <= 0) {
                leftPower = 0.075d - correction;
                rightPower = 0.075d;
            } else {
                leftPower = 0.075d;
                rightPower = 0.075d + correction;
            }

            robot.drive.powerDrive(leftPower, rightPower);
        }
        robot.drive.powerDrive(0,0);

        telemetry.addData("State", "follow line DONE");
        telemetry.update();
        sleep(500);
    }

    private void analyzeBeacon() throws InterruptedException {
        telemetry.addData("State", "analyze beacon");
        telemetry.update();
        sleep(500);

        Beacon.BeaconAnalysis analysis = beacon.getAnalysis();
        double confidence = 0;

        while (!analysis.isBeaconFound() && opModeIsActive()) {
            checkLatestFrame();
            telemetry.addData("Beacon", "not found");
            telemetry.update();
            sleep(100);
        }

        telemetry.addData("Beacon", "found");
        telemetry.update();
        sleep(500);

        while (confidence < BEACON_ANALYSIS_CONFIDENCE && opModeIsActive()) {
            checkLatestFrame();
            confidence = analysis.getConfidence();
            telemetry.addData("Beacon confidence", confidence);
            telemetry.update();
        }

        telemetry.addData("confidence met:", confidence);
        telemetry.update();
        sleep(500);



        if (robot.teamColor == TeamColor.blue) {
            if (analysis.isLeftBlue()) {
                robot.pressBeaconSide("left");
            } else {
                robot.pressBeaconSide("right");
            }
        } else { //team color red
            if (analysis.isLeftRed()) {
                pressButton("left");
            } else {
                pressButton("right");
            }
        }


        pressButton("right");
    }

    private void pressButton(String side) throws InterruptedException {
        telemetry.addData("State", "press button");
        telemetry.update();
        sleep(500);

        robot.pressBeaconSide(side);
        pressBeacon();
        robot.resetBeaconPress();

        telemetry.addData("State", "press button DONE");
        telemetry.update();
        sleep(500);
    }

    private void pressBeacon() throws InterruptedException {
        telemetry.addData("State", "press beacon");
        telemetry.update();
        sleep(500);

        //move to beacon
        double avgHit = robot.ultrasonic.getUltrasonicLevel();

        while (opModeIsActive() && (HIT_BEACON_THRESHOLD <= avgHit) ) {
            avgHit = (avgHit + robot.ultrasonic.getUltrasonicLevel()) / 2;
            robot.drive.powerDrive(0.3, 0.3);
            telemetry.addData("curr distance", robot.ultrasonic.getUltrasonicLevel());
            telemetry.addData("desired distance", HIT_BEACON_THRESHOLD);
            telemetry.addData("avg distance", avgHit);
            telemetry.update();
            sleep(100);
        }
        robot.drive.stop();

        telemetry.addData("Beacon", "pressed");
        telemetry.update();
        sleep(500);

        double avgBack = robot.ultrasonic.getUltrasonicLevel();
        //back up from beacon
        while (opModeIsActive() && (DISTANCE_BEACON_THRESHOLD >= avgBack) ) {
            avgBack = (avgBack + robot.ultrasonic.getUltrasonicLevel()) / 2;
            robot.drive.powerDrive(-0.3, -0.3);
            telemetry.addData("curr distance", robot.ultrasonic.getUltrasonicLevel());
            telemetry.addData("desired distance", DISTANCE_BEACON_THRESHOLD);
            telemetry.addData("avg distance", avgBack);
            telemetry.update();
            sleep(100);
        }

        robot.drive.stop();

        telemetry.addData("backup", "done");
        telemetry.update();
        sleep(500);
    }

    private void checkLatestFrame() {
        //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
        //Vision will run asynchronously (parallel) to any user code so your programs won't hang
        //You can use hasNewFrame() to test whether vision processed a new frame
        //Once you copy the frame, discard it immediately with discardFrame()
        if (hasNewFrame()) {
            //Get the frame
            Mat rgba = getFrameRgba();
            Mat gray = getFrameGray();

            //Discard the current frame to allow for the next one to render
            discardFrame();

            //Do all of your custom frame processing here
            //For this demo, let's just add to a frame counter
            frameCount++;
        }
    }

    private void logVisionData() {
        telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
        telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
        telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
        telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
        telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
        telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
        telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
        telemetry.addData("Frame Counter", frameCount);
    }

    private void stopSB() {

    }
}

