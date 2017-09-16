package org.firstinspires.ftc.teamcode.game.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Robot2 {
    public TeamColor teamColor;
    public StartPosition startPosition;

    public OpticalDistanceSensor ods;
    public UltrasonicSensor ultrasonic;

    public DcMotor  leftMotor;
    public DcMotor  rightMotor;
    public DcMotor  launchMotor;
    public DcMotor collectMotor;

    public Servo    loadServo;
    public Servo    beaconServo;

    public static final double BEACON_UP = 0.6;
    public static final double BEACON_L = 1.5;
    public static final double BEACON_R = 0;

    public static final int LAUNCH_VALUE = 1000;
    public static final double LAUNCH_POWER = 0.6;

    public static final double LOADING_OPEN = 0;
    public static final double LOADING_CLOSE = 0.6;

    /* local OpMode members. */
    private HardwareMap hwMap;

    private Telemetry telemetry;
    private ElapsedTime time;

    public DriveTrain drive;

    public Robot2() {

    }

    public Robot2(TeamColor color, StartPosition pos){
        this.teamColor = color;
        this.startPosition = pos;
    }

    public void setTelemetry(Telemetry t) {
        this.telemetry = t;
    }

    public void setTime(ElapsedTime time) { this.time = time; }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        initHardwareMap(hwMap);
        initDriveTrain();
        initBeaconPress();
        initLaunch();
        initOds();
        initLoading();

        hwMap.logDevices();
    }

    public void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;

        leftMotor   = hwMap.dcMotor.get("leftmotor");
        rightMotor  = hwMap.dcMotor.get("rightmotor");

        launchMotor = hwMap.dcMotor.get("launchmotor");
        collectMotor = hwMap.dcMotor.get("collectmotor");
        collectMotor.setDirection(DcMotor.Direction.FORWARD);

        beaconServo = hwMap.servo.get("beaconservo");
        loadServo = hwMap.servo.get("loadservo");

        ods = hwMap.opticalDistanceSensor.get("ods");
        ultrasonic = hwMap.ultrasonicSensor.get("ultrasonicsensor");
    }

    public void initLaunch() {
        launchMotor.setPower(0);
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initLoading() {
        closeLoading();
    }

    public void openLoading() {
        loadServo.setPosition(LOADING_OPEN);
    }

    public void closeLoading() {
        loadServo.setPosition(LOADING_CLOSE);
    }

    /**
     * BEACON PRESSER
     */

    public void initBeaconPress() {
        resetBeaconPress();
    }

    public void pressBeaconSide(String side) {
        if (side.equals("left")) {
            beaconServo.setPosition(BEACON_L);
        } else {
            beaconServo.setPosition(BEACON_R);
        }
    }

    public void resetBeaconPress() {
        beaconServo.setPosition(BEACON_UP);
    }

    public void initOds() {
        ods.enableLed(false);
    }

    /**
     * DRIVETRAIN
     */

    public void initDriveTrain() {
        drive = new DriveTrain();
    }

    public class DriveTrain {
        static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);

        DcMotor.Direction leftDefaultDir = DcMotor.Direction.FORWARD;
        DcMotor.Direction rightDefaultDir = DcMotor.Direction.REVERSE;

        Queue<PathSeg> paths = new LinkedBlockingQueue();

        PIDController left;
        PIDController right;

        public DriveTrain() {
            leftMotor.setDirection(leftDefaultDir);
            rightMotor.setDirection(rightDefaultDir);

            resetMotors();
            stop();

            left = new PIDController(leftMotor);
            right = new PIDController(rightMotor);
            left.setTimer(time);
            right.setTimer(time);
        }

        public void stop() {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            resetMotors();
        }

        public void resetMotors()  {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void powerDrive(double leftPow, double rightPow) {
            leftMotor.setPower(leftPow);
            rightMotor.setPower(rightPow);
        }

        public void queuePath(PathSeg path) {
            paths.add(path);
        }

        public void startPath() {
            startPath(paths.peek());
        }

        private void startPath(PathSeg path) {
            if (!paths.contains(path)) {
                return;
            }

            resetMotors();

            // Determine new target position, and pass to motor controller
            path.leftTarget = leftMotor.getCurrentPosition() + (int)(path.leftDistance * COUNTS_PER_INCH);
            path.rightTarget = rightMotor.getCurrentPosition() + (int)(path.rightDistance * COUNTS_PER_INCH);

            leftMotor.setTargetPosition(path.leftTarget);
            rightMotor.setTargetPosition(path.rightTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftMotor.setPower(Math.abs(path.speed));
            rightMotor.setPower(Math.abs(path.speed));
        }

        //return true if path done / conditions met, return false if still pathing
        public boolean pathDone() {
            PathSeg path = paths.peek();

            telemetry.addData("Path.clicks.final", "Running to %7d :%7d", path.leftTarget, path.rightTarget);
            telemetry.addData("Path.clicks.current", "Running at L%7d : R%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.update();

            if (!path.isTimedOut()) {
                if (leftMotor.isBusy() && rightMotor.isBusy()) { return false; }
            }

            telemetry.addData("Path-", "finished");
            telemetry.update();
            return true;
        }

        public void stopCurrPath() {
            removePath(paths.peek());
        }

        private void removePath(PathSeg path) {
            if (!paths.contains(path)) {
                return;
            }

            paths.remove(path);
            stop();
        }

    }

}