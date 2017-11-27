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
    public int leftTarget;
    public int rightTarget;

    ElapsedTime runtime;
    double timeOut;
    double startTime;

    static final double timeError = 50; //50 milliseconds

    public PathSeg(double leftDistance,
                   double rightDistance) {

        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        speed = .4;

    }

    public boolean isTimedOut() {
        return isTimedOut(this.runtime);
    }

    public boolean isTimedOut(ElapsedTime currTime) {
        return currTime.milliseconds() - (startTime + timeOut) > timeError;
    }

}