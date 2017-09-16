package org.firstinspires.ftc.teamcode.game.robot;

/**
 * Created by e.xing on 1/7/2017.
 */
public class TeamColor {
    private String color;

    public TeamColor(String c) {
        color = c;
    }

    public String toString() {
        return color;
    }

    public String getType() {
        return color;
    }


    //All types of colors
    public static TeamColor blue = new TeamColor("blue");
    public static TeamColor red = new TeamColor("red");

}

