package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;
import org.firstinspires.ftc.teamcode.game.robot.Robot1;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/**
 * Linear Vision Sample
 * <p/>
 * Use this in a typical linear op mode. A LinearVisionOpMode allows using
 * Vision Extensions, which do a lot of processing for you. Just enable the extension
 * and set its options to your preference!
 * <p/>
 * Please note that the LinearVisionOpMode is specially designed to target a particular
 * version of the FTC Robot Controller app. Changes to the app may break the LinearVisionOpMode.
 * Should this happen, open up an issue on GitHub. :)
 */

//@TeleOp(name="VisionAuto", group="VisionOpmode")
@Disabled
public class VisionAuto extends LinearVisionOpMode {
    Robot1 robot = new Robot1(TeamColor.red, StartPosition.left);
    private ElapsedTime runtime = new ElapsedTime();

    //for drivetrain
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    //ods sensor
    static final double LIGHT_THRESHOLD = 0.5;

    //ultrasonic sensor
    static final double DISTANCE_BEACON_THRESHOLD = 5;

    static boolean stop = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initVision();

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        resetMotors();

        inputGameConfig();

        //Wait for the match to begin
        waitForStart();

        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once the opmode is closed
        int count = 0;
        while (opModeIsActive()) {
            //Log a few things
            /*
            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
            telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
            telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
            telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
            */
            toLaunch();
            launch();
            toBeacon();

            if (lineDetected()) {
                stop = true;
                waitOneFullHardwareCycle();
                followLine();
            }
            analyzeBeacon();
            pressBeacon();
            toLine2();
            if (lineDetected()) {
                stop = true;
                waitOneFullHardwareCycle();
                followLine();
            }
            analyzeBeacon();
            pressBeacon();
            toPushBall();


            if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                Mat gray = getFrameGray();

                //Discard the current frame to allow for the next one to render
                discardFrame();

                //Do all of your custom frame processing here
            }

            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
            telemetry.addData("Count", count);
            count++;
        }
    }

    public void inputGameConfig() throws InterruptedException {
        telemetry.addData("Input team color", "Red (press b) or Blue (press x)");
        telemetry.update();
        while (gamepad1.b != true && gamepad1.x != true) {
            sleep(10);
        }

        if (gamepad1.b == true) {
            robot.teamColor = TeamColor.red;
        } else {
            robot.teamColor = TeamColor.blue;
        }
        telemetry.addData("Chosen team color", robot.teamColor);
        telemetry.update();
        sleep(1000);


        telemetry.addData("Input which side", "Left or right (use triggers)");
        telemetry.update();
        while (!(gamepad1.left_trigger >= 0.5) && !(gamepad1.right_trigger >= 0.5)) {
            sleep(10);
        }

        if (gamepad1.left_trigger >= 0.5) {
            robot.startPosition = StartPosition.left;
        } else {
            robot.startPosition = StartPosition.right;
        }
        telemetry.addData("Chosen start postion", robot.startPosition);
        telemetry.update();
        sleep(1000);
    }

    public void analyzeBeacon() throws InterruptedException {
        Beacon.BeaconAnalysis analysis = beacon.getAnalysis();
        double confidence = analysis.getConfidence();

        while (!(confidence >= 50)) {
            waitOneFullHardwareCycle();
        }

        if (robot.teamColor == TeamColor.blue) {
            if (analysis.isLeftBlue()) {
                pressButton("left");
            } else {
                pressButton("left");
            }
        } else {
            if (analysis.isLeftRed()) {
                pressButton("right");
            } else {
                pressButton("left");
            }
        }
    }


    public void pressButton(String side) throws InterruptedException {
        if (side.equals("left")) {
            robot.beaconLeft();
            pressBeacon();
        }

        if (side.equals("right")) {
            robot.beaconRight();
            pressBeacon();
        }
    }

    public void pressBeacon() throws InterruptedException{
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

            robot.leftMotor.setPower(leftPower);
            robot.rightMotor.setPower(rightPower);

            discardFrame();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        sleep(500);

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

            robot.leftMotor.setPower(leftPower);
            robot.rightMotor.setPower(rightPower);

            discardFrame();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        sleep(500);
    }


    public boolean inFrontOfBeacon() {
        return robot.ultrasonic.getUltrasonicLevel() <= DISTANCE_BEACON_THRESHOLD;
    }

    public boolean lineDetected() {
        telemetry.addData("Light", robot.ods.getLightDetected());
        telemetry.update();
        return robot.ods.getLightDetected() >= LIGHT_THRESHOLD;
    }
    public void toLaunch() throws InterruptedException{
        if(robot.teamColor == TeamColor.red  && robot.startPosition==StartPosition.left){
            encoderdrive(27);
            turnLeft();
            encoderdrive(24);
            turnRight();
            encoderdrive(24);
            turnLeft();
            encoderdrive(15);
        }
        if(robot.teamColor == TeamColor.red  && robot.startPosition==StartPosition.right){
            turnLeft();
            encoderdrive(48);
            turnRight();
            encoderdrive(27);
            turnLeft();
            encoderdrive(24);
            turnRight();
            encoderdrive(24);
            turnLeft();
            encoderdrive(15);
        }
        if(robot.teamColor == TeamColor.blue  && robot.startPosition==StartPosition.right){
            encoderdrive(27);
            turnRight();
            encoderdrive(24);
            turnLeft();
            encoderdrive(24);
            turnRight();
            encoderdrive(15);
        }
        if(robot.teamColor == TeamColor.blue  && robot.startPosition==StartPosition.left){
            turnRight();
            encoderdrive(48);
            turnLeft();
            encoderdrive(27);
            turnRight();
            encoderdrive(24);
            turnLeft();
            encoderdrive(24);
            turnRight();
            encoderdrive(15);
        }
    }
    public void launch() throws InterruptedException{
        robot.launchMotor.setPower(.6);
        sleep(3000);
        robot.launchMotor.setPower(0);
    }
    public void toBeacon() throws InterruptedException{
        turnLeft();
        turnLeft();
    }
    public void toLine2() throws InterruptedException{
        if(robot.teamColor == TeamColor.red){
            encoderdrive(-27);
            turnRight();
            encoderdrive(48);
            turnLeft();
            encoderdrive(24);
        }
        if(robot.teamColor == TeamColor.blue){
            encoderdrive(-27);
            turnLeft();
            encoderdrive(48);
            turnRight();
            encoderdrive(24);
        }
    }
    public void toPushBall() throws InterruptedException{
        if(robot.teamColor == TeamColor.red){
            encoderdrive(-27);
            turnLeft();
            encoderdrive(40);
            turnLeft();
            encoderdrive(36);
        }
        if(robot.teamColor == TeamColor.blue){
            encoderdrive(-27);
            turnRight();
            encoderdrive(40);
            turnRight();
            encoderdrive(36);
        }
    }
    public void encoderdrive(int inches) throws InterruptedException{
        encoderDrive(.6, inches, inches);
    }
    public void turnLeft() throws InterruptedException{
        encoderDrive(.6, -12, 12);
    }
    public void turnRight() throws InterruptedException{
        encoderDrive(.6, 12, -12);
    }
    public void followLine() throws InterruptedException{
        telemetry.addData("follow", 1);
        telemetry.update();
        sleep(1000);

        double leftPower = 0;
        double rightPower = 0;
        while ((!inFrontOfBeacon() || !beacon.getAnalysis().isBeaconFound())&& opModeIsActive()) {
            double correction = (LIGHT_THRESHOLD - robot.ods.getLightDetected());
            if (correction <= 0) {
                leftPower = 0.075d - correction;
                rightPower = 0.075d;
            } else {
                leftPower = 0.075d;
                rightPower = 0.075d + correction;
            }

            robot.leftMotor.setPower(leftPower);
            robot.rightMotor.setPower(rightPower);

            discardFrame();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        analyzeBeacon();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive() && !stop) {
            resetMotors();

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) && !stop) {

                telemetry.addData("Path.final", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path.current", "Running at %7d :%7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                telemetry.update();


            }

            stopMotors();
        }
    }

    public void stopMotors() throws InterruptedException {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void resetMotors() throws InterruptedException {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initVision() throws InterruptedException {
        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();

        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.SECONDARY);

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

        beacon.setAnalysisMethod(Beacon.AnalysisMethod.REALTIME);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

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
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

    }
}
