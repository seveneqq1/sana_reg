package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionAimTrim {
    public static boolean enabled = true;
    public static double gainPerSecond = 2.0;
    public static double maxTrimDegrees = 8.0;
    public static double maxRateDegreesPerSecond = 6.0;
    public static double maxErrorDegrees = 15.0;
    public static double deadbandDegrees = 0.3;
    public static double aimOffsetDegrees = 0.0;
    public static int minimumTags = 1;

    private double degrees;
    private double lastTimestamp = Double.NaN;
    private double lastTime = Double.NaN;

    public double update(double errorDegrees, double timestamp, double nowSeconds) {
        if (!enabled || !Double.isFinite(errorDegrees) || !Double.isFinite(timestamp)
                || !Double.isFinite(nowSeconds)
                || !Double.isFinite(gainPerSecond) || gainPerSecond < 0
                || !Double.isFinite(maxTrimDegrees) || maxTrimDegrees < 0
                || !Double.isFinite(maxRateDegreesPerSecond) || maxRateDegreesPerSecond < 0
                || !Double.isFinite(maxErrorDegrees) || maxErrorDegrees < 0
                || !Double.isFinite(deadbandDegrees) || deadbandDegrees < 0
                || Math.abs(errorDegrees) > maxErrorDegrees) {
            reset();
            return degrees;
        }
        degrees = clamp(degrees, -maxTrimDegrees, maxTrimDegrees);
        if (timestamp == lastTimestamp) return degrees;

        double dt = Double.isNaN(lastTime) ? 0 : clamp(nowSeconds - lastTime, 0, 0.1);
        lastTimestamp = timestamp;
        lastTime = nowSeconds;
        if (Math.abs(errorDegrees) > deadbandDegrees) {
            double maxStep = maxRateDegreesPerSecond * dt;
            degrees += clamp(gainPerSecond * errorDegrees * dt, -maxStep, maxStep);
            degrees = clamp(degrees, -maxTrimDegrees, maxTrimDegrees);
        }
        return degrees;
    }

    public void limitTo(double minimum, double maximum) {
        degrees = clamp(degrees, minimum, maximum);
    }

    public double getDegrees() {
        return degrees;
    }

    public void reset() {
        degrees = 0;
        lastTimestamp = Double.NaN;
        lastTime = Double.NaN;
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
