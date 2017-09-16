package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

/**
 * Created by e.xing on 2/10/2017.
 */
@TeleOp(name="followlinetest", group="testmode")
public class followlinetest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Robot1 robot;

    static final double EDGE_LINE_THRESHOLD = 0.5;
    static final double MID_LINE_THRESHOLD = 0.61;

    public void runOpMode() throws InterruptedException {
        robot = new Robot1();
        robot.init(hardwareMap);
        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //wait for line detected
            while (!lineDetected()) {
                sleep(10);
            }

            telemetry.addData("line", "found");
            telemetry.addData("light data", robot.ods.getLightDetected() +"");
            telemetry.update();
            sleep(1500);

            //follow line
            followLine();
        }


    }

    private void followLine() throws InterruptedException {
        telemetry.addData("State", "follow line");
        telemetry.update();
        sleep(1500);

        double leftPower = 0;
        double rightPower = 0;

        while (opModeIsActive()) {
            double correction = (MID_LINE_THRESHOLD - robot.ods.getLightDetected());

            if (correction <= 0) {
                leftPower = 0.08d - correction;
                rightPower = 0.08d;
            } else {
                leftPower = 0.08d;
                rightPower = 0.08d + correction;
            }

            telemetry.addData("correction", correction + "");
            telemetry.addData("threshold", MID_LINE_THRESHOLD + "");
            telemetry.addData("light detected", robot.ods.getLightDetected() + "");
            telemetry.addData("power:", "L" + leftPower + " R" + rightPower);
            telemetry.update();

            robot.drive.powerDrive(leftPower, rightPower);
        }
        robot.drive.powerDrive(0,0);
    }

    private boolean lineDetected() {
        telemetry.addData("waiting for", "line");
        telemetry.addData("Light", robot.ods.getLightDetected());
        telemetry.update();
        return robot.ods.getLightDetected() >= EDGE_LINE_THRESHOLD;
    }


}


