package org.firstinspires.ftc.teamcode.game.testmodes;

/**
 * Created by 18mjiang on 10/23/17.
 */

/*
 * Copyright (c) 2015 LASA Robotics and Contributors
 * MIT licensed
 */


        import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;
        import org.lasarobotics.vision.android.Cameras;
        import org.lasarobotics.vision.ftc.resq.Beacon;
        import org.lasarobotics.vision.image.Drawing;
        import org.lasarobotics.vision.opmode.TestableVisionOpMode;
        import org.lasarobotics.vision.opmode.VisionOpMode;
        import org.lasarobotics.vision.opmode.extensions.BeaconExtension;
        import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
        import org.lasarobotics.vision.opmode.extensions.ImageRotationExtension;
        import org.lasarobotics.vision.opmode.extensions.VisionExtension;
        import org.lasarobotics.vision.util.ScreenOrientation;
        import org.lasarobotics.vision.util.color.Color;
        import org.lasarobotics.vision.util.color.ColorGRAY;
        import org.lasarobotics.vision.util.color.ColorRGBA;
        import org.opencv.core.Mat;
        import org.opencv.core.Point;
        import org.opencv.core.Size;
        import org.opencv.imgproc.Imgproc;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Vision OpMode run by the Camera Test Activity
 * Use TestableVisionOpModes in testing apps ONLY (but you can easily convert between opmodes just by changingt t
 */
@TeleOp(name="cameratest", group="vision opmode")
public class cameratest extends TestableVisionOpMode {

    /***
     * CUSTOM EXTENSION INITIALIZATION
     * <p/>
     * Add your extension here and in the Extensions class below!
     */
    public static final BeaconExtension beacon = new BeaconExtension();
    public static final ImageRotationExtension rotation = new ImageRotationExtension();
    public static final CameraControlExtension cameraControl = new CameraControlExtension();

    private boolean enableOpenCV = true;
    /**
     * END OF CUSTOM EXTENSION INITIALIZATION
     */

    private int extensions = 0;
    private boolean extensionsInitialized = false;

    public cameratest() {
        super();
    }

    cameratest(boolean enableOpenCV) {
        super();
        this.enableOpenCV = enableOpenCV;
    }

    private boolean isEnabled(VisionOpMode.Extensions extension) {
        return (extensions & extension.id) > 0;
    }

    /**
     * Enable a particular Vision Extension.
     *
     * @param extension Extension ID
     */
    protected void enableExtension(VisionOpMode.Extensions extension) {
        //Don't initialize extension if we haven't ever called init() yet
        if (extensionsInitialized)
            extension.instance.init(this);

        extensions = extensions | extension.id;
    }

    /**
     * Disable a particular Vision Extension
     *
     * @param extension Extension ID
     */
    private void disableExtension(VisionOpMode.Extensions extension) {
        extensions -= extensions & extension.id;

        extension.instance.stop(this);
    }

    @Override
    public void init() {
        if (enableOpenCV) super.init();

        for (VisionOpMode.Extensions extension : VisionOpMode.Extensions.values())
            if (isEnabled(extension))
                extension.instance.init(this);

        extensionsInitialized = true;
        this.setCamera(Cameras.SECONDARY);
        telemetry.addData(Cameras.SECONDARY.name(), Cameras.SECONDARY.getID());
        telemetry.update();
    }

    @Override
    public void loop() {
        if (enableOpenCV) super.loop();

        for (VisionOpMode.Extensions extension : VisionOpMode.Extensions.values())
            if (isEnabled(extension))
                extension.instance.loop(this);
    }

    @Override
    public Mat frame(Mat rgba, Mat gray) {

        for (VisionOpMode.Extensions extension : VisionOpMode.Extensions.values())
            if (isEnabled(extension)) {
                //Pipe the rgba of the previous point into the gray of the next
                Imgproc.cvtColor(rgba, gray, Imgproc.COLOR_RGBA2GRAY);
                extension.instance.frame(this, rgba, gray);
            }

        return rgba;
    }

    @Override
    public void stop() {
        super.stop();

        for (VisionOpMode.Extensions extension : VisionOpMode.Extensions.values())
            if (isEnabled(extension))
                disableExtension(extension); //disable and stop
    }

    /**
     * List of Vision Extensions
     */
    public enum Extensions {
        BEACON(2, beacon),
        CAMERA_CONTROL(1, cameraControl), //high priority
        ROTATION(4, rotation); //low priority

        final int id;
        final VisionExtension instance;

        Extensions(int id, VisionExtension instance) {
            this.id = id;
            this.instance = instance;
        }
    }
}