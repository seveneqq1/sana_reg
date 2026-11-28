package org.firstinspires.ftc.teamcode.bobot.util;

public class HybridAim {
    public static double txGain = 0.1;
    public static double deadbandDeg = 0.75;
    public static double maxOffsetDeg = 25;

    private double offset;
    private double settledSince;

    public double update(double tx, boolean newFrame, boolean atTarget,
                         double frameAgeMs, double nowSeconds) {
        if (!atTarget) settledSince = nowSeconds;
        if (!newFrame) return offset;
        if (!Double.isNaN(tx) && Math.abs(tx) > deadbandDeg
                && (nowSeconds - settledSince) * 1000 > frameAgeMs) {
            offset = Math.max(-maxOffsetDeg, Math.min(maxOffsetDeg, offset + txGain * tx));
            settledSince = nowSeconds;
        }
        return offset;
    }

    public void reset(double nowSeconds) {
        offset = 0;
        settledSince = nowSeconds;
    }

    public double getOffset() { return offset; }
}
