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
import java.util.concurrent.TimeUnit;

import org.lasarobotics.vision.android.Cameras;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="opmode")
public class Auto extends LinearVisionOpMode{
    //variables
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    private final double PERIODIC_INTERVAL = 100; //in milliseconds
    private double nextPeriodicTime;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    //test at tournament in field, configure variables
    static final double LIGHT_THRESHOLD = 0.5; //higher = more light reflected, whiter color
    static final double DISTANCE_BEACON_THRESHOLD = 25;

    static final int BEACON_ANALYSIS_CONFIDENCE = 50;
    static final int GRIP_POS = 3;
    static final int UNGRIP_POS = 5;

    private boolean debugOn = true;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017(TeamColor.red, StartPosition.left);
        robot.init(hardwareMap);
        robot.initDriveTrain();
        robot.drive.resetMotors();
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);
        inputGameConfig();

        if (debugOn) {
            //while (true) { telemetry.addData("Debug:", "init good"); }
        }

        //Wait for the match to begin, presses start button
        waitForStart();

        runtime.reset();


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
            robot.drive.queuePath(new PathSeg(48, 48)); // needs to be path to go to jewels
            robot.drive.startPath();
            while(!robot.drive.pathDone()){

            }
            robot.drive.turnLeft();
            robot.drive.turnRight();
            armForward();
            gripglyph();
            //move here
            analyzeJewels();
            //move here
            placeglyph();
            //Wait for a hardware cycle to allow other processes to run
            waitOneFullHardwareCycle();
        }

    }
    private void armForward() throws InterruptedException{
        robot.armmotor.setPower(.5);
        wait(1400);
        robot.armmotor.setPower(0);
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

    private void gripglyph() throws InterruptedException {
        robot.gripmotor.setPower(-.2);
        wait(1400);
        robot.gripmotor.setPower(0);
        robot.lift1.setPower(.3);
        wait(400);
        robot.lift1.setPower(0);
    }
    private void placeglyph() throws InterruptedException{
        //move up and down, input height could be either 1, 2, 3, or 4 /// well technically we don't need that
        //or we could make constants for each height
        //
        robot.armmotor.setPower(.5);
        wait(600);
        ungripglyph();
        robot.armmotor.setPower(-.5);
        //move back here
    }
    private void ungripglyph() throws InterruptedException{
        robot.gripmotor.setPower(.2);
        wait(200);
        robot.gripmotor.setPower(0);
    }

    private void readpictograph(){
        //back burner
    }

    public void analyzeJewels(){
        int[] rgb = {robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue()};
        if(robot.colorSensor.red()>120){
            if(robot.teamColor.equals(TeamColor.red)){
                robot.drive.turnLeft();
                robot.drive.turnLeft();

            }
            else{
                robot.drive.turnRight();
                robot.drive.turnRight();
            }
        }
        else if(robot.colorSensor.blue()>120){
            if(robot.teamColor.equals(TeamColor.blue)){
                robot.drive.turnLeft();
                robot.drive.turnLeft();

            }
            else{
                robot.drive.turnRight();
                robot.drive.turnRight();
            }
        }
        //ends facing away from jewels
    }
    public android.hardware.Camera initVision(){
        android.hardware.Camera camera = android.hardware.Camera.open(0);

        return camera;
        //make sure to camera.release() after using
    }

}
