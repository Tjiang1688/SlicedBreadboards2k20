package org.firstinspires.ftc.teamcode.testfiles;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import com.qualcomm.robotcore.wifi.NetworkConnection;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

//test for red, left
import android.graphics.Color;

import java.util.EventListener;
import java.util.EventObject;

//@Autonomous(name = "GeneralAuto", group = "Autonomous")
@Disabled
public class TestAuto extends LinearVisionOpMode{
    Robot1 robot = new Robot1();
    float[] color = new float[3];
    final int REDrange1 = 35;
    final int REDrange2 = 290;
    final double WHITE_THRESHOLD = 0.5 ;
    final double RANGE_THRESHOLD = 0.5 ;
    final int LAUNCHTIMEREQUIRED = 3000;
    static final double DISTANCE_BEACON_THRESHOLD = 5;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: andymark Motor Encoder  tetrix is 1440
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932);
    public TestAuto(){

    }

    public void runOpMode() throws InterruptedException{
        boolean opmodechosen = false;
        while(!opmodechosen){
            if(gamepad1.a == true){
                opmodechosen = true;
                robot.teamColor = TeamColor.red;
                robot.startPosition = StartPosition.right;
                telemetry.addLine("red right");

            }
            //
            if(gamepad1.b == true){
                opmodechosen = true;
                robot.teamColor = TeamColor.red;
                robot.startPosition = StartPosition.left;
                telemetry.addLine("red left");

            }
            if(gamepad1.x == true){
                opmodechosen = true;
                robot.teamColor = TeamColor.blue;
                robot.startPosition = StartPosition.left;
                telemetry.addLine("blue left");
            }
            if(gamepad1.y == true){
                opmodechosen = true;
                robot.teamColor = TeamColor.blue;
                robot.startPosition = StartPosition.right;
                telemetry.addLine("blue right");
            }
            robot.init(hardwareMap);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        initVision();
        waitForStart();

        toLaunch();
        launch();
        toBeacon();
        followLine();
        analyzeBeacon();
        toLine2();
        followLine();
        analyzeBeacon();
        toPushBall();
    }
    public void toLaunch() throws InterruptedException{
        if(robot.teamColor==TeamColor.red && robot.startPosition==StartPosition.left){
            encoderdrive(12);
            sleep(150);
            turnLeft();
            sleep(150);
        }
        if(robot.teamColor==TeamColor.red && robot.startPosition==StartPosition.right){
            encoderdrive(12);
            sleep(150);
            turnLeft();
            sleep(150);
            encoderdrive(48);
            sleep(150);

        }
        if(robot.teamColor==TeamColor.blue && robot.startPosition==StartPosition.right){
            encoderdrive(12);
            sleep(150);
            turnLeft();
            sleep(150);
        }
        if(robot.teamColor==TeamColor.blue && robot.startPosition==StartPosition.left){
            encoderdrive(12);
            sleep(150);
            turnRight();
            sleep(150);
            encoderdrive(48);
            sleep(150);
            turnLeft();
            turnLeft();
            sleep(150);
        }

    }
    public void launch() throws InterruptedException{
        robot.launchmotor.setPower(.6);
        sleep(LAUNCHTIMEREQUIRED);
    }
    public void toBeacon() throws InterruptedException{
        if(robot.teamColor == TeamColor.red) {
            encoderdrive(24);
            sleep(150);
            turnRight();
            sleep(150);
            encoderdrive(24);
            sleep(150);
            turnLeft();
            sleep(150);
            encoderdrive(15);
            sleep(150);
        }
        if(robot.teamColor == TeamColor.blue) {
            turnRight();
            turnRight();
            sleep(150);
            encoderdrive(24);
            sleep(150);
            turnLeft();
            sleep(150);
            encoderdrive(24);
            sleep(150);
            turnRight();
            sleep(150);
            encoderdrive(15);
            sleep(150);
        }
    }

    public void followLine(){
        while(robot.ultrasonic.getUltrasonicLevel() > RANGE_THRESHOLD) {
            if (robot.ultrasonic.getUltrasonicLevel() > RANGE_THRESHOLD) {
                robot.leftMotor.setPower(.2);
            } else {
                // Steer left or right
                if (robot.ods.getLightDetected() > WHITE_THRESHOLD) {//lightsensor or ods?
                    robot.rightMotor.setPower(0.2);            // Scan Right

                } else {
                    robot.leftMotor.setPower(0.2);            // Scan Left

                }

            }
        }
    }

    public void analyzeBeacon() throws InterruptedException {
        Beacon.BeaconAnalysis analysis = beacon.getAnalysis();
        double confidence = analysis.getConfidence();

        while (! (confidence >= 50)) {
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
        sleep(300);
    }

    public void pressBeacon() throws InterruptedException {
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
    public void toLine2() throws InterruptedException{
        if(robot.teamColor == TeamColor.red){
            encoderdrive(-27);
            sleep(150);
            turnRight();
            sleep(150);
            encoderdrive(48);
            sleep(150);
            turnLeft();
            sleep(150);
            encoderdrive(24);
            sleep(150);
        }
        if(robot.teamColor == TeamColor.blue){
            encoderdrive(-27);
            sleep(150);
            turnLeft();
            sleep(150);
            encoderdrive(48);
            sleep(150);
            turnRight();
            sleep(150);
            encoderdrive(24);
            sleep(150);
        }
    }
    public void toPushBall() throws InterruptedException{
        if(robot.teamColor == TeamColor.red){
            encoderdrive(-27);
            sleep(150);
            turnLeft();
            sleep(150);
            encoderdrive(40);
            sleep(150);
            turnLeft();
            sleep(150);
            encoderdrive(36);
            sleep(150);
        }
        if(robot.teamColor == TeamColor.blue){
            encoderdrive(-27);
            sleep(150);
            turnRight();
            sleep(150);
            encoderdrive(40);
            sleep(150);
            turnRight();
            sleep(150);
            encoderdrive(36);
            sleep(150);
        }
    }
    public void encoderdrive(int inches) throws InterruptedException{
        encoderDrive(.6, inches, inches, 100);
    }
    public void turnLeft() throws InterruptedException{// or 12? test
        encoderDrive(.6, -13.351769, 13.351769, 100);
    }
    public void turnRight() throws InterruptedException{
        encoderDrive(.6, 13.351769, -13.351769, 100);
    }
    public float[] getHSV(int red, int blue, int green){
        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv;
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

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
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();

            // Allow time for other processes to run.


        }
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
        enableExtension(VisionOpMode.Extensions.BEACON);         //Beacon detection
        enableExtension(VisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control

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
    /*class linedetector implements EventListener{
        public void linedetected(EventObject e){
            followLine();
        }
        public void addEventListener(){
            this.addEventListener();
        }

    }*/

}

/*class PathSeg
{
    public double mLeft;
    public double mRight;
    public double mSpeed;

    // Constructor
    public PathSeg(double Left, double Right, double Speed)
    {
        mLeft = Left;
        mRight = Right;
        mSpeed = Speed;
    }
}*/