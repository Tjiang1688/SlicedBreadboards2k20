package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.robot.PathSeg;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 */
public class Robot2017 {
    //servo positions go from 0 to 1.0, must add suffix to make it a float because Java acts stupid sometimes
    final float jewelservodown = .90f;
    final float jewelservoup = .20f;
    final float griplin = 0.04f;
    final float griprin = .90f;
    final float griplout = .42f;
    final float griprout = .48f;
    public TeamColor teamColor;
    public StartPosition startPosition;
    public DcMotor flMotor;
    public DcMotor frMotor;
    public DcMotor blMotor;
    public DcMotor brMotor;
    public Servo gripl;
    public Servo gripr;
    public DcMotor lift1;
    public DcMotor armmotor;
    public Servo jewelservo;
    public ColorSensor cs;
    public DriveTrain drive;
    private HardwareMap hwMap;
    private Telemetry telemetry;
    private ElapsedTime time;

    public Robot2017() {

    }

    public Robot2017(TeamColor color, StartPosition pos) {
        this.teamColor = color;
        this.startPosition = pos;
    }

    public void setTelemetry(Telemetry t) {
        this.telemetry = t;
    }

    public void setTime(ElapsedTime time) {
        this.time = time;
    }

    public void ungrip() {
        gripl.setPosition(griplout);
        gripr.setPosition(griprout);
    }

    public void grip() {
        gripl.setPosition(griplin);
        gripr.setPosition(griprin);
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        initHardwareMap(hwMap);
        initDriveTrain();
        hwMap.logDevices();
    }

    public void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;

        flMotor = hwMap.dcMotor.get("flmotor");
        frMotor = hwMap.dcMotor.get("frmotor");
        blMotor = hwMap.dcMotor.get("blmotor");
        brMotor = hwMap.dcMotor.get("brmotor");
        lift1 = hwMap.dcMotor.get("liftmotor");
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armmotor = hwMap.dcMotor.get("armmotor");
        jewelservo = hwMap.servo.get("jewelservo");
        gripl = hwMap.servo.get("gripl");
        gripr = hwMap.servo.get("gripr");
        jewelservo.setDirection(Servo.Direction.FORWARD);
        cs = hwMap.colorSensor.get("colorSensor");
        cs.enableLed(true);


    }

    /**
     * DRIVETRAIN
     */

    public void initDriveTrain() {
        drive = new DriveTrain();
    }

    public class DriveTrain {
        static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
        //andymark is 1440 (this needs to be fact-checked)
        static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);
        static final double ROBOT_WIDTH = 17.0;
        static final double TURN_LENGTH = ROBOT_WIDTH * Math.PI / 4;
        DcMotor.Direction leftDefaultDir = DcMotor.Direction.FORWARD;
        DcMotor.Direction rightDefaultDir = DcMotor.Direction.REVERSE;


        Queue<PathSeg> paths = new LinkedBlockingQueue();

        public DriveTrain() {
            resetMotors();
            stop();
        }

        public void stop() {
            flMotor.setPower(0);
            frMotor.setPower(0);
            blMotor.setPower(0);
            brMotor.setPower(0);
            resetMotors();
        }

        public void resetMotors() {
            flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flMotor.setDirection(leftDefaultDir);
            frMotor.setDirection(rightDefaultDir);
            blMotor.setDirection(leftDefaultDir);
            brMotor.setDirection(rightDefaultDir);
        }

        public void horizontal(double length) throws InterruptedException {
            PathSeg right = new PathSeg(-length, length, length, -length, time);
            startPath(right);
            wait1((int) length / 12 * 500);
            wait1(1000);
        }

        public void turnRight() throws InterruptedException {
            PathSeg left = new PathSeg(-2 * TURN_LENGTH, 2 * TURN_LENGTH, -2 * TURN_LENGTH, 2 * TURN_LENGTH, time);
            startPath(left);
            wait1(2000);

        }

        public void turnLeft() throws InterruptedException {
            PathSeg right = new PathSeg(2 * TURN_LENGTH, -2 * TURN_LENGTH, 2 * TURN_LENGTH, -2 * TURN_LENGTH, time);
            startPath(right);
            wait1(2000);
        }

        public void turn(int degree) throws InterruptedException {
            PathSeg turn = new PathSeg(-2 * TURN_LENGTH * degree / 90, 2 * TURN_LENGTH * degree / 90, -2 * TURN_LENGTH * degree / 90, 2 * TURN_LENGTH * degree / 90, time);
            startPath(turn);
            wait1(Math.abs(degree * 10));
            wait1(1000);
        }

        public void vertical(double length) throws InterruptedException {
            PathSeg path = new PathSeg(-length, -length, -length, -length, time);
            startPath(path);
            wait1((int) length / 12 * 500);
            wait1(1000);
        }
       /* public void powerDrive(double leftPow, double rightPow) {
            flMotor.setPower(leftPow);
            frMotor.setPower(rightPow);
            telemetry.addData("Curr power: ", "L" + leftPow + " R" + rightPow);
            telemetry.update();
        }*/

        public void queuePath(PathSeg path) {
            paths.add(path);
        }

        public void startPath() {
            startPath(paths.peek());
        }

        private void startPath(PathSeg path) {

            // Turn On RUN_TO_POSITION
            flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Determine new target position, and pass to motor controller
            path.flTarget = flMotor.getCurrentPosition() + (int) (path.fld * COUNTS_PER_INCH);
            path.frTarget = frMotor.getCurrentPosition() + (int) (path.frd * COUNTS_PER_INCH);
            path.blTarget = blMotor.getCurrentPosition() + (int) (path.bld * COUNTS_PER_INCH);
            path.brTarget = brMotor.getCurrentPosition() + (int) (path.brd * COUNTS_PER_INCH);

            flMotor.setTargetPosition(path.flTarget);
            frMotor.setTargetPosition(path.frTarget);
            blMotor.setTargetPosition(path.blTarget);
            brMotor.setTargetPosition(path.brTarget);

            flMotor.setPower(Math.abs(path.speed));
            frMotor.setPower(Math.abs(path.speed));
            blMotor.setPower(Math.abs(path.speed));
            brMotor.setPower(Math.abs(path.speed));
        }

        //return true if path done / conditions met, return false if still pathing
        public boolean pathDone() {
            PathSeg path = paths.peek();

            telemetry.addData("Path.clicks.final", "Running to %7d :%7d", path.flTarget, path.frTarget, path.blTarget, path.brTarget);
            telemetry.addData("Path.clicks.current", "Running at L%7d : R%7d", flMotor.getCurrentPosition(), frMotor.getCurrentPosition(), blMotor.getCurrentPosition(), brMotor.getCurrentPosition());
            telemetry.addData("path timed out", path.isTimedOut());
            telemetry.update();

            if (!path.isTimedOut()) {
                if (flMotor.isBusy()
                        && frMotor.isBusy()
                        && flMotor.getCurrentPosition() != path.flTarget
                        && frMotor.getCurrentPosition() != path.frTarget) {
                    return false;
                }
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

        private void wait1(int t) throws InterruptedException {
            TimeUnit.MILLISECONDS.sleep(t);
        }
    }
}