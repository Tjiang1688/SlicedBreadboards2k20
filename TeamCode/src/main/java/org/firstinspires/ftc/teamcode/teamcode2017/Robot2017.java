package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.robot.PathSeg;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Robot2017 {
    public TeamColor teamColor;
    public StartPosition startPosition;

    public OpticalDistanceSensor ods;
    public UltrasonicSensor ultrasonic;

    public DcMotor  leftMotor;
    public DcMotor  rightMotor;
    public Servo jewelservo;
    public DcMotor  gripmotor; // possibly a motor
    public DcMotor liftmotor;

    public Servo    griprelic;

    public static final double L_BEACON_UP = 1;
    public static final double L_BEACON_DOWN = 0;

    public DcMotor  relicmotor;

    public static final double BEACON_UP = 0.6;
    public static final double BEACON_L = 1.5;

    /* local OpMode members. */
    private HardwareMap hwMap;

    private Telemetry telemetry;
    private ElapsedTime time;

    public DriveTrain drive;

    public static final double LIGHT_THRESHOLD = 0.5;

    public Robot2017() {

    }

    public Robot2017(TeamColor color, StartPosition pos){
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
        initOds();

        hwMap.logDevices();
    }

    public void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;

        leftMotor   = hwMap.dcMotor.get("leftmotor");
        rightMotor  = hwMap.dcMotor.get("rightmotor");

        gripmotor = hwMap.dcMotor.get("gripmotor");

        ods = hwMap.opticalDistanceSensor.get("ods");
        ultrasonic = hwMap.ultrasonicSensor.get("ultrasonicsensor");
        jewelservo = hwMap.servo.get("jewelservo");
    }

    public void initOds() {
        ods.enableLed(true);
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
            gripmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gripmotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftMotor.setDirection(leftDefaultDir);
            rightMotor.setDirection(rightDefaultDir);
            gripmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
}