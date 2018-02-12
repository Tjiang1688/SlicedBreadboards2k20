package org.firstinspires.ftc.teamcode.teamcode2017;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="driverControlled", group="red")  // @Autonomous(...) is the other common choice
//@Disabled

public class driverControlled extends LinearOpMode {

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
        double[] targets = {0, 0};
        double[] powers = {0, 0};
        //motor power is from -1.0 to 1.0;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("colorsensor", robot.cs.getDeviceName());
        telemetry.update();
        robot.jewelservo.setPosition(robot.jewelservoup);
        while (opModeIsActive()) {

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards
            //float to double, get power from controller
            rightPow = (double) gamepad1.right_stick_y/3*2;
            leftPow = (double) gamepad1.left_stick_y/3*2;
            telemetry.addData("left", "Running to %7d :%7d", robot.flMotor.getCurrentPosition(), robot.blMotor.getCurrentPosition());
            telemetry.addData("right", "Running to %7d :%7d", robot.frMotor.getCurrentPosition(), robot.brMotor.getCurrentPosition());

            //arm
            if(gamepad2.a){
                armPow = .5;

            }
            else if(gamepad2.b){
                //out
                armPow = -.5;

            }
            else{
                armPow = 0;
            }



            //grip
            if(gamepad2.x){
                robot.ungrip();
            }else if(gamepad2.y){
                robot.grip();
            }

            if(gamepad2.dpad_up){
                liftPow = -.4;
            }
            else if(gamepad2.dpad_down){
                liftPow = .2;
            }
            else{
                liftPow = 0;
            }

            if(gamepad2.left_trigger > .5){
                robot.ungrip();
            }
            else if(gamepad2.right_trigger > .5){
             robot.grip();
            }

            if(gamepad1.right_trigger > .5 && robot.jewelservo.getPosition() < .989){
                robot.jewelservo.setPosition(robot.jewelservo.getPosition() + .01);
            }
            else if(gamepad1.right_bumper && robot.jewelservo.getPosition() > .011){
                robot.jewelservo.setPosition(robot.jewelservo.getPosition() - .01);
            }

            /*targets[0] = rightPow;
            targets[1] = leftPow;
            powers[0] = robot.frMotor.getPower();
            powers[1] = robot.flMotor.getPower();
            targets = accel(powers, targets);*/

            telemetry.addData("jewelservo position", robot.jewelservo.getPosition());
            telemetry.addData("jewelservo direction", robot.jewelservo.getDirection());
            telemetry.addData("red", robot.cs.red());
            telemetry.addData("blue", robot.cs.blue());
            telemetry.update();

            robot.flMotor.setPower(leftPow);
            robot.frMotor.setPower(rightPow);
            robot.blMotor.setPower(leftPow);
            robot.brMotor.setPower(rightPow);
            robot.armmotor.setPower(armPow);
            robot.lift1.setPower(liftPow);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    private double[] accel(double[] powers, double[] targets){
        for(int i = 0; i<powers.length; i++){
            if(powers[i]<targets[i]){
                powers[i] += .001;
            }
            else if (powers[i]>targets[i]) {
                powers[i] -= .001;
            }
        }
        return powers;
    }
}
