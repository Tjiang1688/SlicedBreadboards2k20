package org.firstinspires.ftc.teamcode.game.robot;

public class TeamColor {
    //All types of colors
    public static TeamColor blue = new TeamColor("blue");
    public static TeamColor red = new TeamColor("red");
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

}

