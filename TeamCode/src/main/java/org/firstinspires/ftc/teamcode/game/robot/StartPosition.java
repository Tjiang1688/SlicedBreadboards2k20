package org.firstinspires.ftc.teamcode.game.robot;

/**
 * Created by e.xing on 1/7/2017.
 */
public class StartPosition {
    private String position;

    public StartPosition(String p) {
        this.position = p;
    }

    public String toString() {
        return position;
    }

    public String getType() {
        return position;
    }


    //All types of colors
    public static StartPosition left = new StartPosition("left");
    public static StartPosition right = new StartPosition("right");

}
