package org.firstinspires.ftc.teamcode.teamcode2017;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.game.robot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Autonomous", group="red")
public class Auto extends LinearOpMode{
    //variables
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017(TeamColor.red, StartPosition.left);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        inputGameConfig();

        //Wait for the match to begin, presses start button
        waitForStart();
        while (opModeIsActive()) {
            gripglyph();
            wait1(1000);
            robot.jewelservo.setPosition(robot.jewelservodown);
            robot.leftMotor.setPower(.2);
            robot.rightMotor.setPower(.2);

            //while (robot.ods.getLightDetected() > .5){

           // }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            analyzeJewels();
            wait1(1000);
            robot.drive.move(3);



            if(robot.startPosition == StartPosition.left && robot.teamColor == TeamColor.blue){
                robot.drive.turnRight();
                robot.drive.move(48);
                robot.drive.turn(32);
                robot.drive.move(20);
            }

            else if(robot.startPosition == StartPosition.right && robot.teamColor == TeamColor.red){
                robot.drive.turnLeft();
                robot.drive.move(48);
                robot.drive.turn(-32);
                robot.drive.move(20);
            }

            else if(robot.startPosition == StartPosition.right && robot.teamColor == TeamColor.blue){
                robot.drive.turnRight();
                robot.drive.move(21);
                robot.drive.turn(-58);
                robot.drive.move(20);
            }

            else if(robot.startPosition == StartPosition.left && robot.teamColor == TeamColor.red){
                robot.drive.turnLeft();
                robot.drive.move(21);
                robot.drive.turn(58);
                robot.drive.move(20);
            }
            armForward();
            wait1(1000);
            ungripglyph();
            wait1(500);
            robot.lift1.setPower(-.4);
            wait1(600);
            robot.lift1.setPower(0);

            idle();

            wait1(30000);

        }

    }
    private void armForward() throws InterruptedException{
        robot.armmotor.setPower(-.5);
        wait1(1400);
        robot.armmotor.setPower(0);
    }
    private void inputGameConfig() throws InterruptedException{
        telemetry.addData("Input team color", "Red (press b) or Blue (press x)");
        telemetry.update();
        while (!gamepad1.b && !gamepad1.x) {
        }

        if (gamepad1.b == true) {
            robot.teamColor = TeamColor.red;
        } else {
            robot.teamColor = TeamColor.blue;
        }
        telemetry.addData("Chosen team color", robot.teamColor);

        telemetry.addData("Input which side", "Left or right (use triggers)");
        telemetry.update();
        while (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5) {
        }

        if (gamepad1.left_trigger >= 0.5) {
            robot.startPosition = StartPosition.left;
        } else {
            robot.startPosition = StartPosition.right;
        }
        telemetry.addData("Chosen start postion", robot.startPosition);
        telemetry.update();
    }

    private void gripglyph() throws InterruptedException {
        robot.gripmotor.setPower(.2);
        wait1(600);
        robot.gripmotor.setPower(0);
        robot.lift1.setPower(-.4);
        wait1(400);
        robot.lift1.setPower(0);
    }
    private void ungripglyph() throws InterruptedException{
        robot.gripmotor.setPower(-.2);
        wait1(200);
        robot.gripmotor.setPower(0);
    }

    private void readpictograph(){
        //back burner
    }
    public void wait1(int t) throws InterruptedException{
        TimeUnit.MILLISECONDS.sleep(t);
    }
    public void analyzeJewels() throws InterruptedException{

        wait1(2000);
        telemetry.addData("red", robot.cs.red());
        telemetry.addData("blue", robot.cs.blue());
        telemetry.update();
        int count = 0;
        while(robot.cs.red() == 0 && robot.cs.blue() == 0){
            robot.drive.turn(1);
            count++;
        }
        if(robot.cs.red()>robot.cs.blue()){
            telemetry.addData("ball color", "red");
            if(robot.teamColor.equals(TeamColor.red)){
                telemetry.addData("turning left", "");
                robot.drive.turn(-30);
                wait1(1000);
                robot.jewelservo.setPosition(robot.jewelservoup);
                wait1(1000);
                robot.drive.turn(30);
                wait1(1000);
            }
            else{
                telemetry.addData("turning right", "");
                robot.drive.turn(30);
                wait1(1000);
                robot.jewelservo.setPosition(robot.jewelservoup);
                wait1(1000);
                robot.drive.turn(-30);
                wait1(1000);
            }
        }
        else if(robot.cs.red()<robot.cs.blue()){
            telemetry.addData("ball color", "red");
            if(robot.teamColor.equals(TeamColor.blue)){
                telemetry.addData("turning left", "");
                robot.drive.turn(-30);
                wait1(1000);
                robot.jewelservo.setPosition(robot.jewelservoup);
                wait1(1000);
                robot.drive.turn(30);
                wait1(1000);
            }
            else{
                telemetry.addData("turning right", "");
                robot.drive.turn(30);
                wait1(1000);
                robot.jewelservo.setPosition(robot.jewelservoup);
                wait1(1000);
                robot.drive.turn(-30);
                wait1(1000);
            }
        }
        robot.drive.turn(-count);
        //ends facing away from jewels
    }
    public android.hardware.Camera initVision(){
        android.hardware.Camera camera = android.hardware.Camera.open(0);

        return camera;
        //make sure to camera.release() after using
    }

}
