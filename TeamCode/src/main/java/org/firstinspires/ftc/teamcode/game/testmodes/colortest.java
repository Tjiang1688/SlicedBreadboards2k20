package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.teamcode2017.Robot2017;
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
@TeleOp(name="colortest", group="testmode")
public class colortest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Robot2017 robot;

    static final double EDGE_LINE_THRESHOLD = 0.5;
    static final double MID_LINE_THRESHOLD = 0.61;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.initDriveTrain();
        robot.teamColor = TeamColor.red;
        robot.startPosition = StartPosition.left;

        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //wait for line detected
            while(true){
                telemetry.addData("blue", robot.cs.blue());
                telemetry.addData("red", robot.cs.red());
                telemetry.update();
            }

        }


    }


}


