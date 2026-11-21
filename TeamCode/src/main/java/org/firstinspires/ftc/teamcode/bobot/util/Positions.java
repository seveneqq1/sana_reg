package org.firstinspires.ftc.teamcode.bobot.util;

import com.pedropathing.math.Pose;

public class Positions {
    public static Pose CURRENT;

    public Positions(Pose CURRENT) {
        this.CURRENT = CURRENT;
    }

    public static void setCURRENT(Pose CURRENT) {
        Positions.CURRENT = CURRENT;
    }

    public static Pose getCURRENT() {
        return CURRENT;
    }
}
