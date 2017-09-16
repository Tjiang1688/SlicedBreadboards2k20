package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.game.robot.*;

/**
 * Created by e.xing on 2/10/2017.
 */
@TeleOp(name="beaconpresstest", group="testmode")
public class beaconpresstest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        Robot1 robot = new Robot1();
        robot.init(hardwareMap);

        waitForStart();
        runtime.reset();

        robot.resetBeaconPress();
        telemetry.addData("press side", robot.beaconServo.getVersion());
        sleep(1500);

        while (opModeIsActive()) {
            telemetry.addData("press side:", "left");
            telemetry.update();
            robot.pressBeaconSide("left");

            sleep(1000);

            telemetry.addData("current pos", robot.beaconServo.getPosition());
            telemetry.update();

            sleep(1000);

            telemetry.addData("press side:", "right");
            telemetry.update();
            robot.pressBeaconSide("right");

            sleep(1000);

            telemetry.addData("current pos", robot.beaconServo.getPosition());
            telemetry.update();

            sleep(1000);

            break;
        }


    }
}
