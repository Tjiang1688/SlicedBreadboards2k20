package org.firstinspires.ftc.teamcode.game.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Robot1 {
    public TeamColor teamColor;
    public StartPosition startPosition;

    public OpticalDistanceSensor ods;
    public UltrasonicSensor ultrasonic;

    public DcMotor  leftMotor;
    public DcMotor  rightMotor;

    public DcMotor  launchmotor;
    public DcMotor collectmotor;

    public Servo    beaconservo;

    public static final double L_BEACON_UP = 1;
    public static final double L_BEACON_DOWN = 0;

    public DcMotor  launchMotor;
    public DcMotor collectMotor;

    public Servo    loadServo;
    public Servo    beaconServo;

    public static final double BEACON_UP = 0.6;
    public static final double BEACON_L = 1.5;
    public static final double BEACON_R = 0;

    /* local OpMode members. */
    private HardwareMap hwMap;

    private Telemetry telemetry;
    private ElapsedTime time;

    public DriveTrain drive;
    public Launcher launcher;

    public static final double LIGHT_THRESHOLD = 0.5;

    public Robot1() {

    }

    public Robot1(TeamColor color, StartPosition pos){
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
        initOds();
        initLauncher();

        hwMap.logDevices();
    }

    public void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;

        leftMotor   = hwMap.dcMotor.get("leftmotor");
        rightMotor  = hwMap.dcMotor.get("rightmotor");

        launchMotor = hwMap.dcMotor.get("launchmotor");
        collectMotor = hwMap.dcMotor.get("collectmotor");
        collectMotor.setDirection(DcMotor.Direction.FORWARD);

        beaconservo = hwMap.servo.get("beaconservo");

        beaconServo = hwMap.servo.get("beaconservo");
        loadServo = hwMap.servo.get("loadservo");

        ods = hwMap.opticalDistanceSensor.get("ods");
        ultrasonic = hwMap.ultrasonicSensor.get("ultrasonicsensor");
    }

    public void initLauncher() {
        launcher = new Launcher(launchMotor, loadServo);
    }

    public void initOds() {
        ods.enableLed(true);
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
        //beaconServo.setPosition(BEACON_UP);
    }

    public void beaconRight(){
        beaconservo.setPosition(1);
    }
    public void beaconLeft() {
        beaconservo.setPosition(0);
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

        public DriveTrain() {
            resetMotors();
            stop();
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
            leftMotor.setDirection(leftDefaultDir);
            rightMotor.setDirection(rightDefaultDir);
        }

        public void powerDrive(double leftPow, double rightPow) {
            leftMotor.setPower(leftPow);
            rightMotor.setPower(rightPow);
            telemetry.addData("Curr power: ", "L" + leftPow + " R" + rightPow);
            telemetry.update();
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

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Determine new target position, and pass to motor controller
            path.leftTarget = leftMotor.getCurrentPosition() + (int)(path.leftDistance * COUNTS_PER_INCH);
            path.rightTarget = rightMotor.getCurrentPosition() + (int)(path.rightDistance * COUNTS_PER_INCH);

            leftMotor.setTargetPosition(path.leftTarget);
            rightMotor.setTargetPosition(path.rightTarget);

            leftMotor.setPower(Math.abs(path.speed));
            rightMotor.setPower(Math.abs(path.speed));
        }

        //return true if path done / conditions met, return false if still pathing
        public boolean pathDone() {
            PathSeg path = paths.peek();

            telemetry.addData("Path.clicks.final", "Running to %7d :%7d", path.leftTarget, path.rightTarget);
            telemetry.addData("Path.clicks.current", "Running at L%7d : R%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.addData("path timed out", path.isTimedOut());
            telemetry.update();

            if (!path.isTimedOut()) {
                if (leftMotor.isBusy()
                        && rightMotor.isBusy()
                        && leftMotor.getCurrentPosition() != path.leftTarget
                        && rightMotor.getCurrentPosition() != path.rightTarget) { return false; }
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

    public class Launcher {
        public static final int LOAD_TIME = 800; //in milliseconds
        public static final int LAUNCH_TIME = 1000;

        public static final int READY_LAUNCH_POS = -(1120 + 600);
        public static final int FIRE_LAUNCH_POS = (int) (-1120 * 3.6);

        public static final double LAUNCH_POWER = 0.6;

        public static final double LOADING_OPEN = 0;
        public static final double LOADING_CLOSE = 0.6;


        DcMotor launch;
        Servo load;

        public Launcher(DcMotor motor, Servo servo) {
            launch = motor;
            load = servo;

            launch.setDirection(DcMotor.Direction.REVERSE);
            reset();
        }

        public void reset() {
            closeLoading();
            launch.setPower(0);

            launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void openLoading() {
            load.setPosition(LOADING_OPEN);
        }

        public void closeLoading() {
            load.setPosition(LOADING_CLOSE);
        }

        public void stop() {
            launch.setPower(0);
        }

        public void fire() {
            launch.setTargetPosition(FIRE_LAUNCH_POS);
            launch.setPower(LAUNCH_POWER);
        }

        public void ready() {
            launch.setTargetPosition(READY_LAUNCH_POS);
            launch.setPower(LAUNCH_POWER);
        }
    }
}