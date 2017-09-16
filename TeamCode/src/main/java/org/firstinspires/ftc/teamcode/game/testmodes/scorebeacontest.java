package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/**
 * Created by e.xing on 2/10/2017.
 */
@TeleOp(name="scorebeacontest", group="testmode")
public class scorebeacontest extends LinearVisionOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    /***
     * VISION VARIABLES
     */

    private Cameras BEACON_CAMERA = Cameras.PRIMARY;
    private Beacon.AnalysisMethod BEACON_ANALYSIS_METHOD = Beacon.AnalysisMethod.REALTIME;
    private ScreenOrientation SCREEN_ORIENTATION = ScreenOrientation.LANDSCAPE;

    Robot1 robot;

    static final double EDGE_LINE_THRESHOLD = 0.45;
    static final double MID_LINE_THRESHOLD = 0.85;

    static final double DISTANCE_BEACON_THRESHOLD = 35;
    static final double HIT_BEACON_THRESHOLD = 8;

    static final double BEACON_ANALYSIS_CONFIDENCE = 0.75;


    public void runOpMode() throws InterruptedException {
        initVision();

        robot = new Robot1();
        robot.teamColor = TeamColor.red;
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //wait for line detected
            while (!lineDetected()) {
                sleep(100);
            }

            //follow line
            followLine();

            //analyze beacon
            analyzeBeacon();

            break;
        }

        telemetry.addData("beacon", "scored");
        telemetry.update();
        sleep(1000);

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

        double leftPower = 0;
        double rightPower = 0;

        //move to beacon
        double avgHit = robot.ultrasonic.getUltrasonicLevel();

        while (opModeIsActive() && (HIT_BEACON_THRESHOLD <= avgHit) ) {
            avgHit = (avgHit + robot.ultrasonic.getUltrasonicLevel()) / 2;
            robot.drive.powerDrive(0.45, 0.45);
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
            robot.drive.powerDrive(-0.45, -0.405);
            telemetry.addData("curr distance", robot.ultrasonic.getUltrasonicLevel());
            telemetry.addData("desired distance", DISTANCE_BEACON_THRESHOLD);
            telemetry.addData("avg distance", avgBack);
            telemetry.update();
            sleep(100);
        }


        robot.drive.stop();

        telemetry.addData("backup", "done");
        telemetry.update();
    }

    private void followLine() throws InterruptedException {
        telemetry.addData("State", "follow line");
        telemetry.update();
        sleep(500);

        double leftPower = 0;
        double rightPower = 0;

        while (!inFrontOfBeacon()
                && opModeIsActive()) {
            double correction = (MID_LINE_THRESHOLD - robot.ods.getLightDetected()) / 1.05 ;
            if (correction <= 0) {
                leftPower = 0.05d;
                rightPower = 0.05d - correction;
            } else {
                leftPower = 0.05d + correction;
                rightPower = 0.05d;
            }

            robot.drive.powerDrive(leftPower, rightPower);
        }
        robot.drive.powerDrive(0,0);

        telemetry.addData("State", "follow line DONE");
        telemetry.update();
        sleep(500);
    }

    private boolean inFrontOfBeacon() throws InterruptedException {
        telemetry.addData("ultrasonic status", robot.ultrasonic.getUltrasonicLevel());
        telemetry.update();
        sleep(500);
        return robot.ultrasonic.getUltrasonicLevel() <= DISTANCE_BEACON_THRESHOLD;
    }

    private boolean lineDetected() {
        telemetry.addData("waiting for", "line");
        telemetry.addData("Light", robot.ods.getLightDetected());
        telemetry.update();
        return robot.ods.getLightDetected() > EDGE_LINE_THRESHOLD;
    }

    private void initVision() throws InterruptedException {
        waitForVisionStart();
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
    }

}
