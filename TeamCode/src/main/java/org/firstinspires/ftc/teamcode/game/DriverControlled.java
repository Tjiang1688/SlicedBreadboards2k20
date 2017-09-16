package org.firstinspires.ftc.teamcode.game;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.game.robot.*;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OldDriverControlled", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class DriverControlled extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    Robot1 robot = new Robot1();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)
        waitForStart();
        runtime.reset();
        //Activate beacon presser
        boolean collect = false;
        boolean launch = false;

        double leftPow = 0;
        double rightPow = 0;

        //motor power is from -1.0 to 1.0;
        // change in pow
        double lball = 0;

        double quantize = 0.1;
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards

            //float to double, get power from controller
            double desiredLeftPow = (double) gamepad1.left_stick_y;
            double desiredRightPow = (double) gamepad1.right_stick_y;

            //normalize values
            double maxDesiredPow = Math.max(Math.abs(desiredLeftPow), Math.abs(desiredRightPow));
            if (maxDesiredPow > 1.0)
            {
                desiredLeftPow /= maxDesiredPow;
                desiredRightPow /= maxDesiredPow;
            }

            robot.beaconservo.setPosition(0.1);

            //truncates values
            desiredLeftPow = truncate(desiredLeftPow);
            desiredRightPow = truncate(desiredRightPow);
            System.out.println("dLnorm:" + desiredLeftPow + " dRnorm:" + desiredRightPow);

            System.out.println("!" + leftPow);
            if (desiredLeftPow == 0 ) {
                leftPow += Math.signum(leftPow) * -quantize;
            } else if (Math.abs(desiredLeftPow) - Math.abs(leftPow) != 0) {
                leftPow += Math.signum(desiredLeftPow) * quantize;
            }
            System.out.println("!" + leftPow);

            if (desiredRightPow == 0) {
                rightPow += Math.signum(rightPow) * -quantize;
            } else if (Math.abs(desiredRightPow) - Math.abs(rightPow) != 0) {
                rightPow += Math.signum(desiredRightPow) * quantize;
            }

            //normalizes motor power values
            double maxPow = Math.max(Math.abs(leftPow), Math.abs(rightPow));
            if (maxPow > 1.0)
            {
                leftPow /= maxPow;
                rightPow /= maxPow;
            }

            leftPow = truncate(leftPow);
            rightPow = truncate(rightPow);

            robot.leftMotor.setPower(leftPow);
            robot.rightMotor.setPower(rightPow);

            //collect
            if (gamepad1.a == true) {
                collect = true;
            } else if (gamepad1.b == true) {
                collect = false;
            }

            if (collect == true) {
                robot.collectMotor.setPower(1);
            } else {
                robot.collectMotor.setPower(0);
            }

            //launch
            if (gamepad1.x == true) {
                launch = true;
            } else if (gamepad1.y == true) {
                launch = false;
            }

            if (launch == true && lball <= 1500) {
                robot.launchMotor.setPower(0.85);
                lball++;
            } else if (lball > 1500) {
                launch = false;
                lball = 0;
            } else {
                robot.launchMotor.setPower(0);
                lball = 0;
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void power(double desiredLeftPow, double desiredRightPow, double leftPow, double rightPow, double quantize) {
            //normalizes desiredPower values
            double maxDesiredPow = Math.max(Math.abs(desiredLeftPow), Math.abs(desiredRightPow));
            if (maxDesiredPow > 1.0)
            {
                desiredLeftPow /= maxDesiredPow;
                desiredRightPow /= maxDesiredPow;
            }

            //truncates values
            desiredLeftPow = truncate(desiredLeftPow);
            desiredRightPow = truncate(desiredRightPow);
            System.out.println("dLnorm:" + desiredLeftPow + " dRnorm:" + desiredRightPow);

            System.out.println("!" + leftPow);
            if (desiredLeftPow == 0 ) {
                leftPow += Math.signum(leftPow) * -quantize;
            } else if (Math.abs(desiredLeftPow) - Math.abs(leftPow) != 0) {
                leftPow += Math.signum(desiredLeftPow) * quantize;
            }
            System.out.println("!" + leftPow);

            if (desiredRightPow == 0) {
                rightPow += Math.signum(rightPow) * -quantize;
            } else if (Math.abs(desiredRightPow) - Math.abs(rightPow) != 0) {
                rightPow += Math.signum(desiredRightPow) * quantize;
            }

            //normalizes motor power values
            double maxPow = Math.max(Math.abs(leftPow), Math.abs(rightPow));
            if (maxPow > 1.0)
            {
                leftPow /= maxPow;
                rightPow /= maxPow;
            }

            leftPow = truncate(leftPow);
            rightPow = truncate(rightPow);
    }


    // takes in a val with valMin and valMax, scales to a number in range limMin to limMax
    public double scale(double val, double valMin, double valMax, double limMin, double limMax) {
        return ((limMax - limMin) * (val - valMin) / (valMax - valMin)) + valMin;
    }

    public double truncate(double val) {
        if (val < 0) {
            val = 0.1 * Math.ceil(val * 10.0);
        } else {
            val = 0.1 * Math.floor(val * 10.0);
        }
        return val;
    }
    public void settoloadposition(){
        robot.launchmotor.setTargetPosition(3);

    }
}
