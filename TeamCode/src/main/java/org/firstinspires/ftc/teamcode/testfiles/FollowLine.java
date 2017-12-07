package org.firstinspires.ftc.teamcode.testfiles;

/**
 * Created by 18mjiang on 1/13/17.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.robot.Robot1;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

//@Autonomous(name = "FollowLineTest", group = "Autonomous")
@Disabled
public class FollowLine extends LinearOpMode {
    Robot1 robot;
    final double WHITE_THRESHOLD = 0.5;
    final double RANGE_THRESHOLD = 0.5;

    public FollowLine() {

    }

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new Robot1();
        robot.teamColor = TeamColor.red;
        robot.startPosition = StartPosition.left;
        robot.init(hardwareMap);
        waitForStart();
        followLine();
    }

    public void followLine() {
        while(robot.ultrasonic.getUltrasonicLevel() > RANGE_THRESHOLD) {
            if (robot.ultrasonic.getUltrasonicLevel() > RANGE_THRESHOLD) {
                robot.leftMotor.setPower(.2);
            } else {
                // Steer left or right
                if (robot.ods.getLightDetected() > WHITE_THRESHOLD) {//lightsensor or ods?
                    robot.rightMotor.setPower(0.2);            // Scan Right

                } else {
                    robot.leftMotor.setPower(0.2);            // Scan Left

                }

            }
        }
    }
}