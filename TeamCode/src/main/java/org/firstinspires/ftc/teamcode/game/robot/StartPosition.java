package org.firstinspires.ftc.teamcode.game.robot;

public class StartPosition {
    //All types of colors
    public static StartPosition left = new StartPosition("left");
    public static StartPosition right = new StartPosition("right");
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

}
