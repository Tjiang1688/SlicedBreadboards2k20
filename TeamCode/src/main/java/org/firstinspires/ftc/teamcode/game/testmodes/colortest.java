package org.firstinspires.ftc.teamcode.game.testmodes;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;

@TeleOp(name="colortest", group="testmode")
public class colortest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    static final double EDGE_LINE_THRESHOLD = 0.5;
    static final double MID_LINE_THRESHOLD = 0.61;

    public void runOpMode() throws InterruptedException {
        HiTechnicNxtColorSensor cs = (HiTechnicNxtColorSensor) hardwareMap.colorSensor.get("colorSensor");
        cs.enableLed(true);
        cs.enableLight(true);
        telemetry.addData("Light on?", cs.isLightOn());
        telemetry.addData("init", "done");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            //wait for line detected
            while(true){
                float[] hsv = new float[3];
                
                        Color.RGBToHSV(cs.red(), cs.green(), cs.blue(), hsv);

                // send the info back to driver station using telemetry function.\
                telemetry.addData("ARGB", cs.argb());
                telemetry.addData("Clear", cs.alpha());
                telemetry.addData("Red  ", cs.red());
                telemetry.addData("Green", cs.green());
                telemetry.addData("Blue ", cs.blue());
                telemetry.addData("Hue", hsv[0]);
                telemetry.addData("blue", cs.blue());
                telemetry.addData("red", cs.red());
                telemetry.update();
            }

        }


    }


}


