package org.firstinspires.ftc.teamcode.game.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    DcMotor motor;

    double prevTime;
    double setPoint;
    double prevInput; //derivative kick, change from dErr, http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/

    double errSum, prevErr;
    double kp, ki, kd;

    int controllerDirection; // -1 for reverse, http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/

    double iTerm; //tuning changes, http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

    double minOut, maxOut; //reset windup, http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/

    ElapsedTime currTime;
    int cycleTime = 500; //500ms

    public PIDController(DcMotor motor) {
        this.motor = motor;
        this.setPoint = motor.getTargetPosition();

        if (motor.getDirection().equals(DcMotor.Direction.FORWARD)) {
            controllerDirection = 1;
        } else if (motor.getDirection().equals(DcMotor.Direction.REVERSE)) {
            controllerDirection = -1;
        }
    }

    public double compute(double input) {
        double nowTime = currTime.milliseconds();
        double deltaTime = nowTime - prevTime;

        if (!(deltaTime >= cycleTime)) {
            return -1;
        }

        double error = setPoint - input;
        iTerm += (ki * error * deltaTime);

        //correct for integral windup by setting a maximum
        //also known as reset windup
        if (iTerm > maxOut) {
            iTerm = maxOut;
        } else if (iTerm < minOut) {
            iTerm = minOut;
        }

        //double dErr = (error - prevErr) / deltaTime;
        double dInput = (input - prevInput) / deltaTime;

        double output = kp * error + iTerm + kd * dInput;

        if (output > maxOut) {
            output = maxOut;
        } else if (output < minOut) {
            output = minOut;
        }

        prevInput = input;
        prevErr = error;
        prevTime = nowTime;

        return output;
    }

    public void setOutputLimits(double min, double max) {
        if (min > max) return;

        minOut = min;
        maxOut = max;
    }

    public void setConstants(double kp, double ki, double kd) {
        if (kp < 0 || ki < 0 || kd < 0) {
            return;
        }

        this.kp = kp * controllerDirection;
        this.ki = ki * controllerDirection;
        this.kd = kd * controllerDirection;
    }

    public void setDirection(int direction) {
        controllerDirection = direction;
    }

    public void setTimer(ElapsedTime time) {
        this.currTime = time;
    }

    public void setCycleTime(int n) {
        cycleTime = n;
    }

}
