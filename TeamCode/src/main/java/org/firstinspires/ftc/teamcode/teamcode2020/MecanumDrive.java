package org.firstinspires.ftc.teamcode.teamcode2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumDrive", group = "Drive")  // @Autonomous(...) is the other common choice
//@Disabled

public class MecanumDrive extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Robot2017 robot = new Robot2017();
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)
        waitForStart();
        runtime.reset();
        //double armPow = 0;
        double liftPow = 0;
        double[] targets = {0, 0, 0, 0};
        double[] powers = {0, 0, 0, 0};
        //motor power is from -1.0 to 1.0;
        telemetry.addData("Status", "Initialized");
        //telemetry.addData("colorsensor", robot.cs.getDeviceName());
        telemetry.update();
        //robot.jewelservo.setPosition(robot.jewelservoup);
        //robot.gripl.setPosition(.5);
        //robot.gripr.setPosition(.5);
        while (opModeIsActive()) {
            double rightX = gamepad1.right_stick_x;
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double v1 = leftY - leftX - rightX;
            double v2 = leftY + leftX + rightX;
            double v3 = leftY + leftX - rightX;
            double v4 = leftY - leftX + rightX;

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards
            //float to double, get power from controller
            telemetry.addData("left", "Running to %7d :%7d", robot.flMotor.getCurrentPosition(), robot.blMotor.getCurrentPosition());
            telemetry.addData("right", "Running to %7d :%7d", robot.frMotor.getCurrentPosition(), robot.brMotor.getCurrentPosition());



            if (gamepad2.dpad_down) {
                liftPow = -0.75;
            } else if (gamepad2.dpad_up) {
                liftPow = 0.75;
            } else if (gamepad2.a) {
                liftPow = -0.5;
            } else if (gamepad2.y) {
                liftPow = 0.5;
            } else {
                liftPow = 0;
            }

            if (gamepad2.right_trigger > .5 && robot.markerServo.getPosition() < 0.7994) {
                robot.markerServo.setPosition(robot.markerServo.getPosition() + .01);
            } else if (gamepad2.right_bumper && robot.markerServo.getPosition() > 0.4698) {
                robot.markerServo.setPosition(robot.markerServo.getPosition() - .01);
            }

            if (gamepad2.left_trigger > .5 && robot.pulleyHolder.getPosition() < 0.7983) {
                robot.pulleyHolder.setPosition(robot.pulleyHolder.getPosition() + .01);
            } else if (gamepad2.left_bumper && robot.pulleyHolder.getPosition() > 0.5083) {
                robot.pulleyHolder.setPosition(robot.pulleyHolder.getPosition() - .01);
            }

            powers[0] = robot.frMotor.getPower();
            powers[1] = robot.flMotor.getPower();
            powers[2] = robot.brMotor.getPower();
            powers[3] = robot.blMotor.getPower();
            targets = accel(powers, targets);

            //telemetry.addData("gripl", robot.gripl.getPosition());
            //telemetry.addData("gripr", robot.gripr.getPosition());
            //telemetry.addData("red", robot.cs.red());
            //telemetry.addData("blue", robot.cs.blue());
            telemetry.addData("Marker Servo Position: ", robot.markerServo.getPosition());
            telemetry.addData("Pulley Holder Position: ", robot.pulleyHolder.getPosition());
            telemetry.update();

            if(gamepad1.y){
                robot.flMotor.setPower(v1/2);
                robot.frMotor.setPower(v2/2);
                robot.blMotor.setPower(v3/2);
                robot.brMotor.setPower(v4/2);
            } else {
                robot.flMotor.setPower(v1);
                robot.frMotor.setPower(v2);
                robot.blMotor.setPower(v3);
                robot.brMotor.setPower(v4);
            }

            robot.liftMotor.setPower(liftPow);
            robot.negLiftMotor.setPower(-liftPow);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    private double[] accel(double[] powers, double[] targets) {
        for (int i = 0; i < powers.length; i++) {
            if (powers[i] < targets[i]) {
                powers[i] += .001;
            } else if (powers[i] > targets[i]) {
                powers[i] -= .001;
            }
        }
        return powers;
    }
}
