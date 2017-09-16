package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.*;

/**
 * Created by e.xing on 12/7/2016.
 */
@TeleOp(name="encodertest", group="testmode")
public class encodertest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);


    public void runOpMode() throws InterruptedException {

        Robot1 robot = new Robot1();
        robot.init(hardwareMap);
        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //in inches
        double leftDistance = 48.0;
        double rightDistance = 48.0;

        int leftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftDistance * COUNTS_PER_INCH);
        int rightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightDistance * COUNTS_PER_INCH);

        double robotPower = 0.3;

        robot.leftMotor.setTargetPosition(leftTarget); //6 feet
        robot.rightMotor.setTargetPosition(rightTarget);

        robot.leftMotor.setPower(robotPower);
        robot.rightMotor.setPower(robotPower);

        while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Desired path (L, R):", "%7d :%7d",
                    leftTarget,
                    rightTarget);

            telemetry.addData("Current path (L, R):", "%7d :%7d",
                    robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
            sleep(500);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
}



