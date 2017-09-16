package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.*;

/**
 * Created by e.xing on 12/7/2016.
 */
@TeleOp(name="ultrasonictest", group="testmode")
public class ultrasonictest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        Robot1 robot = new Robot1();
        robot.init(hardwareMap);
        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            telemetry.addData("ultrasonic val:", robot.ultrasonic.getUltrasonicLevel());

            telemetry.addData("status", robot.ultrasonic.status());
            telemetry.update();

            sleep(1000);
        }


    }
}