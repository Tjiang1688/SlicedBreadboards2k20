package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.Robot2;

/**
 * Created by e.xing on 2/10/2017.
 */
@TeleOp(name="pidtest", group="testmode")
public class pidtest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    double nextPeriodicTime = 0;
    public static final double PERIODIC_INTERVAL = 500;

    Robot2 robot;

    public void runOpMode() throws InterruptedException {

        robot = new Robot2();
        robot.init(hardwareMap);

        robot.setTime(runtime);
        robot.setTelemetry(telemetry);
        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();

        startSB();

        nextPeriodicTime = runtime.milliseconds();

        double distance = 48; //go 48 inches

        while (opModeIsActive()) {
            if (runtime.milliseconds() >= nextPeriodicTime) {
                nextPeriodicTime += PERIODIC_INTERVAL;
                periodicLoop();
            }
        }

    }

    public void periodicLoop() {

    }

    public void startSB() {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

