package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

    public DcMotor  leftMotor;
    public DcMotor  rightMotor;
    public DcMotor  gripmotor; // possibly a motor
    public DcMotor lift1;
    public DcMotor lift2;
    public DcMotor armmotor;
    //public ColorSensor colorSensor;
    public Servo jewelservo;
    public HiTechnicNxtColorSensor cs;
    /* local OpMode members. */
    private HardwareMap hwMap;
    private Telemetry telemetry;
    private ElapsedTime time;

    public DriveTrain drive;
    final float jewelservodown = 1;
    final float jewelservoup = 0;

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

        hwMap.logDevices();
    }

    public void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;

        leftMotor   = hwMap.dcMotor.get("leftmotor");
        rightMotor  = hwMap.dcMotor.get("rightmotor");

        gripmotor = hwMap.dcMotor.get("gripmotor");
        gripmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripmotor.setDirection(DcMotor.Direction.FORWARD);
        lift1 = hwMap.dcMotor.get("lift1");
        lift2 = hwMap.dcMotor.get("lift2");
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        armmotor = hwMap.dcMotor.get("armmotor");
        jewelservo = hwMap.servo.get("jewelservo");
        //colorSensor = hwMap.colorSensor.get("colorSensor");
        //colorSensor.enableLed(true);
        //check if can cast over
        cs = (HiTechnicNxtColorSensor) hwMap.colorSensor.get("colorSensor");
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
        static final double     ROBOT_WIDTH = 17.0;
        static final double     TURN_LENGTH = ROBOT_WIDTH*Math.PI/4;
        DcMotor.Direction leftDefaultDir = DcMotor.Direction.REVERSE;
        DcMotor.Direction rightDefaultDir = DcMotor.Direction.FORWARD;

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
        public void turnLeft() {
            PathSeg left = new PathSeg(-TURN_LENGTH, TURN_LENGTH);
            queuePath(left);
            startPath();
            while(!pathDone()){

            }
        }
        public void turnRight(){
            PathSeg left = new PathSeg(TURN_LENGTH, -TURN_LENGTH);
            queuePath(left);
            startPath();
            while(!pathDone()){

            }
        }
        public void move(double length){
            PathSeg path = new PathSeg(length, length);
            queuePath(path);
            startPath();
            while(!pathDone()){

            }
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