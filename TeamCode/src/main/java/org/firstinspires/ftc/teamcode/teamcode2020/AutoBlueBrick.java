package org.firstinspires.ftc.teamcode.teamcode2020;

/**
 * Created by 22tjiang on 9/13/19.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.game.robot.Convert;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name = "AutoBlueBrick", group = "Auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AutoBlueBrick extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        double distTravelled = 2.0;

        //*inputGameConfig();


        //Wait for the match to begin, presses start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        while (opModeIsActive()) {
            robot.composeIMUTelemetry();
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(2), robot.getHeading());
            robot.gyrodrive.turn(0.7, -90);
            telemetry.addData("arrive at", "bricks");

            /**

            if (color_sensor.argb() > 100){
                robot.gyrodrive.vertical(0.7, .5, robot.getHeading());
            }
            **/

            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(distTravelled), robot.getHeading());
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-distTravelled), robot.getHeading());
            robot.gyrodrive.turn(0.7, -180);
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(1), robot.getHeading());










        }
    }
}