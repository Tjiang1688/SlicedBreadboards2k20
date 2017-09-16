package org.firstinspires.ftc.teamcode.testfiles;

/**
 * Created by e.xing on 1/6/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;

public class EncoderTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    Robot1 robot = new Robot1();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        runtime.reset();

        robot.leftMotor.setPower(1.0);
        robot.rightMotor.setPower(1.0);

        while (opModeIsActive() && (runtime.time() < 10)) {
            telemetry.addData("EncoderL", "%4.1f : %d coumts", runtime.time(), robot.leftMotor.getCurrentPosition());
            telemetry.addData("EncoderR", "%4.1f : %d coumts", runtime.time(), robot.rightMotor.getCurrentPosition());

            telemetry.update();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        telemetry.addData("Encoder", "%5.0f Counts per second", (double) (robot.leftMotor.getCurrentPosition()) / runtime.time());
        telemetry.addData("Encoder", "%5.0f Counts per second", (double) (robot.rightMotor.getCurrentPosition()) / runtime.time());

        telemetry.update();
        while (opModeIsActive());
    }

}
