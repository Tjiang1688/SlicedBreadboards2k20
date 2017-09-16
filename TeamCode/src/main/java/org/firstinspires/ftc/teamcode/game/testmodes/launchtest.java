package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;

/**
 * Created by e.xing on 2/9/2017.
 */
@TeleOp(name="launchtest", group="testmode")
public class launchtest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        Robot1 robot = new Robot1();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int readyLaunch = -(1120 + 600);
        int inactiveLaunch = (int) (-1120 * 3.6);
        double launchPower = 0.5;

        double readyLoad = 0;
        double inactiveLoad = 0.75;

        int loadTime = 400;

        //set target pos to negative, its going positive, so take - of currpos to get rel value
        while (opModeIsActive()) {
            //activate launch
            robot.launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.launchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.launchMotor.setTargetPosition(readyLaunch);
            robot.launchMotor.setPower(launchPower);
            while (robot.launchMotor.isBusy() && opModeIsActive() && -robot.launchMotor.getCurrentPosition() > readyLaunch) {
                telemetry.addData("ready launch, curr pos", robot.launchMotor.getCurrentPosition());
                telemetry.addData("desired pos", readyLaunch);
                telemetry.update();
                sleep(100);
            }
            robot.launchMotor.setPower(0);

            telemetry.addData("launch is:", "ready");
            telemetry.update();
            sleep(1500);

            //load ball
            robot.launcher.openLoading();
            sleep(loadTime);
            robot.launcher.closeLoading();

            telemetry.addData("ball is:", "loaded");
            telemetry.update();
            sleep(1500);

            //launch ball
            robot.launchMotor.setTargetPosition(inactiveLaunch);
            robot.launchMotor.setPower(launchPower);
            while (robot.launchMotor.isBusy() && opModeIsActive() && -robot.launchMotor.getCurrentPosition() > inactiveLaunch) {
                telemetry.addData("fire launch pos", robot.launchMotor.getCurrentPosition());
                telemetry.addData("desired pos", inactiveLaunch);
                telemetry.update();
                sleep(10);
            }
            robot.launchMotor.setPower(0);

            telemetry.addData("final curr pos", robot.launchMotor.getCurrentPosition());
            telemetry.addData("desired pos", inactiveLaunch);
            telemetry.update();
            sleep(1500);

            break;
        }


    }

}