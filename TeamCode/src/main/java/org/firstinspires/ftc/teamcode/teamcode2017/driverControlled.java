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
        robot.initDriveTrain();
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)
        waitForStart();
        runtime.reset();
        double leftPow = 0;
        double rightPow = 0;
        double armPow = 0;
        double gripPow = 0;
        double liftPow = 0;
        double lift2Pow = 0;
        //motor power is from -1.0 to 1.0;
        telemetry.addData("Status", "Initialized");
        telemetry.addData("colorsensor", robot.cs.getDeviceName());
        telemetry.update();
        while (opModeIsActive()) {

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards
            //float to double, get power from controller
            rightPow = (double) gamepad1.right_stick_y;
            leftPow = (double) gamepad1.left_stick_y;
            telemetry.addData("left", robot.leftMotor.getCurrentPosition());
            telemetry.addData("right", robot.rightMotor.getCurrentPosition());
            if(gamepad1.a){
                armPow = .5;

            }
            else if(gamepad1.b){
                //out
                armPow = -.5;

            }
            else{
                armPow = 0;
            }

            if(gamepad1.x){
                //ungrip
                gripPow = .2;
            }else if(gamepad1.y){
                gripPow = -.2;
            }
            else{
                gripPow = 0;
            }

            if(gamepad1.dpad_up){
                liftPow = -.4;
            }
            else if(gamepad1.dpad_down){
                liftPow = .2;
            }
            else{
                liftPow = 0;
            }

            if(gamepad1.left_trigger> .5){
                robot.jewelservo.setPosition(robot.jewelservo.getPosition() + .01);
            }
            else if(gamepad1.right_trigger > .5){
                robot.jewelservo.setPosition(robot.jewelservo.getPosition() - .01);
            }
            telemetry.addData("jewelservo position", robot.jewelservo.getPosition());
            if(gamepad1.dpad_right){
                lift2Pow = -.4;
            }
            else if(gamepad1.dpad_left){
                lift2Pow = .2;
            }
            else {
                lift2Pow = 0;
            }
            telemetry.addData("grip", robot.lift2.getCurrentPosition());
            telemetry.addData("red", robot.cs.red());
            telemetry.addData("green", robot.cs.green());
            telemetry.addData("blue", robot.cs.blue());

            telemetry.update();

            robot.leftMotor.setPower(leftPow);
            robot.rightMotor.setPower(rightPow);
            robot.armmotor.setPower(armPow);
            robot.gripmotor.setPower(gripPow);

            robot.lift1.setPower(liftPow);
            robot.lift2.setPower(lift2Pow);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    private double min(double a, double b){
        if(Math.abs(a) > Math.abs(b)){
            return b;
        }
        else if(Math.abs(a) < Math.abs(b)){
            return a;
        }
        return a;
    }
}
