package org.firstinspires.ftc.teamcode.teamcode2017;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="driverControlled", group="red")  // @Autonomous(...) is the other common choice
//@Disabled
public class driverControlled extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("x to accelerate, y to deccelerate", "");
        telemetry.update();
        Robot2017 robot = new Robot2017();
        robot.init(hardwareMap);
        robot.initDriveTrain();
        robot.drive.resetMotors();
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)
        waitForStart();
        runtime.reset();
        double leftPow = 0;
        double rightPow = 0;
        double armPow = 0;
        //motor power is from -1.0 to 1.0;
        double quantize = 0.1;
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards
            //float to double, get power from controller
            double lowerPow = Math.min(leftPow, rightPow);
            armPow = 0;
            if(gamepad1.x){
                rightPow += .001;
                leftPow += .001;
                if(rightPow > 1.0){
                    rightPow = 1.0;
                }
                if(leftPow > 1.0){
                    leftPow = 1.0;
                }
            }
            else if(gamepad1.y){
                rightPow -= .001;
                leftPow -= .001;
                if(rightPow < -1.0){
                    rightPow = -1.0;
                }
                if(leftPow < -1.0){
                    leftPow = -1.0;
                }
            }
            else{
                if(lowerPow < .05 || lowerPow > -.05){
                    rightPow = 0;
                    leftPow = 0;
                }
                else if(lowerPow<0){
                    rightPow += .001;
                    leftPow += .001;

                }
                else if(lowerPow>0){
                    rightPow -= .001;
                    leftPow -= .001;

                }
            }
            if(gamepad1.a){
                armPow = .5;
                telemetry.addData("Current position", robot.armmotor.getCurrentPosition());
            }
            if(gamepad1.b){
                armPow = -.5;
                telemetry.addData("Current position", robot.armmotor.getCurrentPosition());
            }
            if(gamepad1.dpad_left){
                rightPow += .001;
                leftPow -= .001;
            }
            else if(gamepad1.dpad_right){
                rightPow -= .001;
                leftPow += .001;
            }
            else{
                rightPow = min(leftPow, rightPow);
                leftPow = rightPow;
            }

            robot.leftMotor.setPower(leftPow);
            robot.rightMotor.setPower(rightPow);
            robot.armmotor.setPower(armPow);
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
