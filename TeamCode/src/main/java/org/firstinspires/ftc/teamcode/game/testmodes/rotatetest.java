package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.PathSeg;
import org.firstinspires.ftc.teamcode.game.robot.Robot1;

/**
 * Created by e.xing on 2/10/2017.
 */
@TeleOp(name="rotatetest", group="testmode")
public class rotatetest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        Robot1 robot = new Robot1();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //robot.drive.queuePath(new PathSeg(14, -14, 0.45, runtime, 10000));
            robot.drive.queuePath(new PathSeg(14 * 1.2, -14 * 1.2, 0.35, runtime, 10000));
            robot.drive.startPath();

            while (!robot.drive.pathDone() && opModeIsActive()) {
                sleep(10);
            }
            robot.drive.stopCurrPath();

            telemetry.addData("rotate", "done");
            telemetry.update();
            sleep(1000);

            robot.drive.queuePath(new PathSeg(12 * 6.7, 12 * 6.7, 0.5, runtime, 15000));
            robot.drive.startPath();

            while (!robot.drive.pathDone() && opModeIsActive()) {
                sleep(10);
            }
            robot.drive.stopCurrPath();

            telemetry.addData("path", "done");
            telemetry.update();
            sleep(1000);

            while (robot.ods.getLightDetected() < robot.LIGHT_THRESHOLD && opModeIsActive()){
                robot.drive.powerDrive(0.1, -0.1);
            }
            robot.drive.powerDrive(0, 0);

            telemetry.addData("line", "found");
            telemetry.update();
            sleep(1000);

            break;
        }

    }
}