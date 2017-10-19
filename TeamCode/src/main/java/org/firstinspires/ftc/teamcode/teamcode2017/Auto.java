package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.game.robot.PathSeg;
import org.firstinspires.ftc.teamcode.teamcode2017.Robot2017;
import org.firstinspires.ftc.teamcode.game.robot.*;
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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="opmode")
public class Auto extends LinearVisionOpMode{
    //variables
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    private final double PERIODIC_INTERVAL = 100; //in milliseconds
    private double nextPeriodicTime;

    //test at tournament in field, configure variables
    static final double LIGHT_THRESHOLD = 0.5; //higher = more light reflected, whiter color
    static final double DISTANCE_BEACON_THRESHOLD = 25;

    static final int BEACON_ANALYSIS_CONFIDENCE = 50;
    static final int GRIP_POS = 3;
    static final int UNGRIP_POS = 5;

    private boolean debugOn = true;

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
            }
            //queue all the paths here, in order
            robot.drive.queuePath(new PathSeg(48, 48, runtime, 10000)); // needs to be path to go to jewels


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

        inputGameConfig();
    }
    private void startSB() {
        runtime.reset();
    }
    private void stopSB() {

    }
    private void gripglyph(){
        robot.gripmotor.setTargetPosition(GRIP_POS);
        robot.gripmotor.setPower(1.0);
    }
    private void placeglyph(float height){
        //move up and down, input height could be either 1, 2, 3, or 4
        //or we could make constants for each height
        //

    }
    private void ungripglyph(){
        robot.gripmotor.setTargetPosition(GRIP_POS);
        robot.gripmotor.setPower(1.0);
    }
    private void jewelcolor(){
        //see
    }
    private void jewelknock(boolean side){ //true = left, false = right
        //PathSeg forward, need to get field measurements, etc
        //servo moving side to side
        robot.jewelservo.setPosition(1); //base off of side
    }
    private void move(){
        //don't necessarily need this
        //need to test PathSeg and startpath() in Robot2017 class
    }
    private void readpictograph(){
        //back burner
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
