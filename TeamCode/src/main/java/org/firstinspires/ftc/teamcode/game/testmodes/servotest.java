package org.firstinspires.ftc.teamcode.game.testmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="servotest", group="red")  // @Autonomous(...) is the other common choice
//@Disabled

public class servotest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)
        waitForStart();
        Servo left = hardwareMap.servo.get("gripl");
        Servo right = hardwareMap.servo.get("gripr");

        while (opModeIsActive()) {

            if(gamepad1.a && left.getPosition() < .989){
                left.setPosition(left.getPosition() + .01);
            }else if (gamepad1.b && left.getPosition() > .011){
                left.setPosition(left.getPosition() - .01);
            }
            if(gamepad1.x && right.getPosition() < .989){
                right.setPosition(right.getPosition() + .01);
            }else if (gamepad1.y && left.getPosition() > .011){
                right.setPosition(right.getPosition() - .01);
            }


            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards
            //float to double, get power from controller
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
