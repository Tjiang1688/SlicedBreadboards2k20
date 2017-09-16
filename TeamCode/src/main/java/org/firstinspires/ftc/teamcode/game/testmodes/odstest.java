package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.*;

/**
 * Created by e.xing on 12/7/2016.
 */
@TeleOp(name="odstest", group="testmode")
public class odstest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        Robot1 robot = new Robot1();
        robot.init(hardwareMap);
        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            telemetry.addData("max val", robot.ods.getRawLightDetectedMax());
            telemetry.addData("raw val", robot.ods.getRawLightDetected());
            telemetry.addData("scale val", robot.ods.getLightDetected());

            telemetry.addData("status", robot.ods.status());
            telemetry.update();

            sleep(1000);
        }


    }
}



