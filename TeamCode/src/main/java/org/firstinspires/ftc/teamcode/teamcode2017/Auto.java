package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Autonomous", group = "red")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
//if it still doesn't work just change to annotate to TeleOp again -- pretty sure there aren't regulations about this
public class Auto extends LinearOpMode {
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
            armForward();
            analyzeJewels();
            wait1(1000);

            if (robot.startPosition == StartPosition.left && robot.teamColor == TeamColor.blue) {
                robot.drive.vertical(-24);
                robot.drive.turn(180);
                robot.drive.horizontal(12);
                robot.drive.vertical(6);
                robot.ungrip();
                wait1(1000);
                robot.drive.vertical(-3);
            } else if (robot.startPosition == StartPosition.right && robot.teamColor == TeamColor.red) {
                robot.drive.vertical(24);
                robot.drive.horizontal(-12);
                robot.drive.vertical(6);
                robot.ungrip();
                wait1(1000);
                robot.drive.vertical(-3);
            } else if (robot.startPosition == StartPosition.right && robot.teamColor == TeamColor.blue) {
                robot.drive.vertical(-36);
                robot.drive.turnRight();
                robot.drive.vertical(6);
                robot.ungrip();
                wait1(1000);
                robot.drive.vertical(-3);
            } else if (robot.startPosition == StartPosition.left && robot.teamColor == TeamColor.red) {
                robot.drive.vertical(36);
                robot.drive.turnRight();
                robot.drive.vertical(6);
                robot.ungrip();
                wait1(1000);
                robot.drive.vertical(-3);
            }
            wait1(500);
            idle();
            wait1(30000);
        }
    }

    private void armForward() throws InterruptedException {
        robot.armmotor.setPower(-.5);
        wait1(1400);
        robot.armmotor.setPower(0);
    }

    private void inputGameConfig() throws InterruptedException {
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
        robot.grip();
        robot.lift1.setPower(.2);
        wait1(400);
        robot.lift1.setPower(0);
    }

    private void readpictograph() {
        //back burner
    }

    public void wait1(int t) throws InterruptedException {
        TimeUnit.MILLISECONDS.sleep(t);
    }

    public void analyzeJewels() throws InterruptedException {

        wait1(2000);
        int count = 0;
        while (robot.cs.red() == 0 && robot.cs.blue() == 0) {
            robot.drive.vertical(-.2);
            count++;
        }
        int red = robot.cs.red();
        int blue = robot.cs.blue();
        if (red > blue) {
            telemetry.addData("saw red", "sdlkfjslkfd");
            telemetry.update();
            if (robot.teamColor.equals(TeamColor.red)) {
                robot.drive.vertical(5);
                wait1(1000);
                robot.jewelservo.setPosition(robot.jewelservoup);
                wait1(1000);
                robot.drive.vertical(-5);
                wait1(1000);
            } else {
                robot.drive.vertical(-5);
                wait1(1000);
                robot.jewelservo.setPosition(robot.jewelservoup);
                wait1(1000);
                robot.drive.vertical(5);
                wait1(1000);
            }
        } else if (red < blue) {
            telemetry.addData("saw blue", "sdkfk");
            telemetry.update();
            if (robot.teamColor.equals(TeamColor.blue)) {
                robot.drive.vertical(5);
                wait1(1000);
                robot.jewelservo.setPosition(robot.jewelservoup);
                wait1(1000);
                robot.drive.vertical(-5);
                wait1(1000);
            } else {
                robot.drive.vertical(-5);
                wait1(1000);
                robot.jewelservo.setPosition(robot.jewelservoup);
                wait1(1000);
                robot.drive.vertical(5);
                wait1(1000);
            }
        } else {
            robot.jewelservo.setPosition(robot.jewelservoup);
        }
        robot.drive.vertical(.2 * count);
    }

    public android.hardware.Camera initVision() {
        android.hardware.Camera camera = android.hardware.Camera.open(0);

        return camera;
        //make sure to camera.release() after using
    }

}
