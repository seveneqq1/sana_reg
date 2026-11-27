package org.firstinspires.ftc.teamcode.bobot.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RegShooter {
    public static boolean calibrated = true;
    public static boolean useQuadratic = true;
    public static double quadraticA = 0.332561;
    public static double quadraticB = -30.533292;
    public static double quadraticC = 1939.17245;
    public static double slopeTPSPerInch = 11.02715;
    public static double interceptTPS = 722.1;

    public static double velForDist(double distanceInches, double fixedTPS) {
        if (!Double.isFinite(distanceInches) || distanceInches < 0) return 0;

        double targetTPS = fixedTPS;
        if (calibrated) {
            targetTPS = useQuadratic
                    ? (quadraticA * distanceInches + quadraticB) * distanceInches + quadraticC
                    : slopeTPSPerInch * distanceInches + interceptTPS;
        }
        return Double.isFinite(targetTPS) && targetTPS > 0 ? targetTPS : 0;
    }
}
