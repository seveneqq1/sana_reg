package org.firstinspires.ftc.teamcode.bobot.util;

public final class ShooterDistance {
    private ShooterDistance() {}

    public static double targetTPS(double distanceInches) {
        if (!Double.isFinite(distanceInches) || distanceInches < 0) return 0;
        return 5.84216 * distanceInches + 1049.03115 - 25;
    }
}
