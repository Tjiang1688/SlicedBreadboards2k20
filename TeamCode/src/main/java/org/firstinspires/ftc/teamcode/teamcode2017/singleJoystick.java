package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "singleJoystick", group = "red")  // @Autonomous(...) is the other common choice
//@Disabled

public class singleJoystick extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Robot2017 robot = new Robot2017();
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)
        waitForStart();
        runtime.reset();
        double leftPow = 0;
        double rightPow = 0;
        double armPow = 0;
        double liftPow = 0;
        double pow = 0;
        double angle = 0;
        double x = 0;
        double y = 0;
        //motor power is from -1.0 to 1.0;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("colorsensor", robot.cs.getDeviceName());
        telemetry.update();
        while (opModeIsActive()) {

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards
            //float to double, get power from controller
            x = gamepad1.right_stick_x;
            y = gamepad1.right_stick_y;
            angle = Math.toDegrees(Math.atan(y / x));
            pow = Math.sqrt((y * y) + (x * x));
            if (x > 0) {
                leftPow = pow;
                rightPow = angle / 90 * pow;
            } else if (x < 0) {
                rightPow = pow;
                leftPow = angle / 90 * pow;
            } else {
                rightPow = pow;
                leftPow = pow;
            }

            if (y < 0) {
                leftPow = -leftPow;
                rightPow = -rightPow;
            }

            if (gamepad1.a) {
                armPow = .5;

            } else if (gamepad1.b) {
                //out
                armPow = -.5;

            } else {
                armPow = 0;
            }

            if (gamepad1.x) {
                robot.ungrip();
            } else if (gamepad1.y) {
                robot.grip();
            }


            if (gamepad1.dpad_up) {
                liftPow = -.4;
            } else if (gamepad1.dpad_down) {
                liftPow = .2;
            } else {
                liftPow = 0;
            }

            if (gamepad1.left_trigger > .5) {
                robot.jewelservo.setPosition(robot.jewelservodown);
            } else if (gamepad1.right_trigger > .5) {
                robot.jewelservo.setPosition(robot.jewelservoup);
            }

            telemetry.addData("jewelservo position", robot.jewelservo.getPosition());
            telemetry.addData("jewelservo direction", robot.jewelservo.getDirection());
            telemetry.addData("red", robot.cs.red());
            telemetry.addData("green", robot.cs.green());
            telemetry.addData("blue", robot.cs.blue());

            telemetry.update();

            robot.flMotor.setPower(leftPow);
            robot.frMotor.setPower(rightPow);
            robot.armmotor.setPower(armPow);

            robot.lift1.setPower(liftPow);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    private double[] accel(double[] powers, double[] targets) {
        for (int i = 0; i < powers.length; i++) {
            if (i < 2) {
                if (powers[i] < targets[i] - .0003) {
                    powers[i] += .0003;
                } else if (powers[i] > targets[i] + .0003) {
                    powers[i] -= .0003;
                }
            }
        }
        return powers;
    }
}
