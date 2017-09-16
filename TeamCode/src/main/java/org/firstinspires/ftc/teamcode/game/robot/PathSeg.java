package org.firstinspires.ftc.teamcode.game.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */
public class PathSeg {
    //desired distance for each motor
    public double leftDistance;
    public double rightDistance;

    //desired speed
    public double speed;

    //encoder tick target for motors
    int leftTarget;
    int rightTarget;

    ElapsedTime runtime;
    double timeOut;
    double startTime;

    static final double timeError = 50; //50 milliseconds

    public PathSeg(double leftDistance,
                   double rightDistance,
                   double speed,
                   ElapsedTime runtime,
                   double timeOut) {

        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        this.speed = speed;

        this.runtime = runtime;
        this.timeOut = timeOut;
        this.startTime = runtime.milliseconds();
    }

    public boolean isTimedOut() {
        return isTimedOut(this.runtime);
    }

    public boolean isTimedOut(ElapsedTime currTime) {
        return currTime.milliseconds() - (startTime + timeOut) > timeError;
    }

}