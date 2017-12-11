package org.firstinspires.ftc.teamcode.teamcode2017;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.game.robot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Autonomous", group="red")
public class Auto extends LinearOpMode{
    //variables
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    private final double PERIODIC_INTERVAL = 100; //in milliseconds
    private double nextPeriodicTime;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017(TeamColor.red, StartPosition.left);
        robot.init(hardwareMap);
        robot.initDriveTrain();
        robot.drive.resetMotors();
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);
        inputGameConfig();

        //Wait for the match to begin, presses start button
        waitForStart();

        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once the opmode is closed
        while (opModeIsActive()) {
            gripglyph();
            wait1(1500);

            robot.drive.move(36);
            if(robot.startPosition == StartPosition.right){
                robot.drive.turnLeft();
                telemetry.addData("turning left", "team right");
            }
            else if(robot.startPosition == StartPosition.left){
                robot.drive.turnRight();
                telemetry.addData("turning right", "team left");
            }
            telemetry.update();
            wait1(1000);
            robot.drive.move(8);
            armForward();
            wait1(1000);
            //move here
            //analyzeJewels();
            //move here
            ungripglyph();
            wait1(10000);
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
        robot.gripmotor.setPower(-.2);
        wait1(600);
        robot.gripmotor.setPower(0);
        robot.lift1.setPower(-.4);
        wait1(400);
        robot.lift1.setPower(0);
    }
    private void ungripglyph() throws InterruptedException{
        robot.gripmotor.setPower(.2);
        wait1(200);
        robot.gripmotor.setPower(0);
    }

    private void readpictograph(){
        //back burner
    }
    public void wait1(int t) throws InterruptedException{
        TimeUnit.MILLISECONDS.sleep(t);
    }
/*public void analyzeJewels(){
        int[] rgb = {robot.cs.red(), robot.cs.green(), robot.cs.blue()};
        if(robot.cs.red()>15){
            if(robot.teamColor.equals(TeamColor.red)){
                robot.drive.turnLeft();
                robot.drive.turnLeft();

            }
            else{
                robot.drive.turnRight();
                robot.drive.turnRight();
            }
        }
        else if(robot.cs.blue()>15){
            if(robot.teamColor.equals(TeamColor.blue)){
                robot.drive.turnLeft();
                robot.drive.turnLeft();

            }
            else{
                robot.drive.turnRight();
                robot.drive.turnRight();
            }
        }
        //ends facing away from jewels
    }*/
    public android.hardware.Camera initVision(){
        android.hardware.Camera camera = android.hardware.Camera.open(0);

        return camera;
        //make sure to camera.release() after using
    }

}
