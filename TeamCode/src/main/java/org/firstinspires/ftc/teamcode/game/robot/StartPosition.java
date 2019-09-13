package org.firstinspires.ftc.teamcode.game.robot;

public class StartPosition {
    public static StartPosition marker = new StartPosition("marker");
    public static StartPosition crater = new StartPosition("crater");
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
