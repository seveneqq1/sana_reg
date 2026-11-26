package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterEncoderFeedback {
    public static double minimumFeedbackTPS = 100;
    public static double switchDelaySeconds = 0.25;
    public static double noFeedbackTimeoutSeconds = 1.5;
    public static double minimumCommandedPower = 0.2;

    private boolean usingLeft = true;
    private boolean fault;
    private double suspectSince = Double.NaN;
    private double noFeedbackSince = Double.NaN;
    private double previousTime = Double.NaN;
    private double velocity = Double.NaN;
    private double acceleration;
    private double leftTPS = Double.NaN;
    private double rightTPS = Double.NaN;

    public double update(double leftTPS, double rightTPS, double targetTPS,
                         double commandedPower, double nowSeconds) {
        this.leftTPS = Math.abs(leftTPS);
        this.rightTPS = Math.abs(rightTPS);
        boolean previousSource = usingLeft;
        boolean enabled = Double.isFinite(targetTPS) && targetTPS > 0;
        boolean powered = enabled && commandedPower >= positiveOrDefault(minimumCommandedPower, 0.2);
        double threshold = Math.min(positiveOrDefault(minimumFeedbackTPS, 100), targetTPS * 0.2);
        boolean leftHealthy = Double.isFinite(this.leftTPS) && this.leftTPS >= threshold;
        boolean rightHealthy = Double.isFinite(this.rightTPS) && this.rightTPS >= threshold;
        double selected = usingLeft ? this.leftTPS : this.rightTPS;
        double backup = usingLeft ? this.rightTPS : this.leftTPS;

        if (!fault) {

            if (!Double.isFinite(selected) && Double.isFinite(backup)) {
                usingLeft = !usingLeft;
                suspectSince = Double.NaN;
            } else if (powered && !(usingLeft ? leftHealthy : rightHealthy)
                    && (usingLeft ? rightHealthy : leftHealthy)) {
                if (!Double.isFinite(suspectSince)) suspectSince = nowSeconds;
                if (nowSeconds - suspectSince >= positiveOrDefault(switchDelaySeconds, 0.25)) {
                    usingLeft = !usingLeft;
                    suspectSince = Double.NaN;
                }
            } else {
                suspectSince = Double.NaN;
            }

            if (enabled && !Double.isFinite(this.leftTPS) && !Double.isFinite(this.rightTPS)) {
                fault = true;
            } else if (powered && !leftHealthy && !rightHealthy) {
                if (!Double.isFinite(noFeedbackSince)) noFeedbackSince = nowSeconds;
                fault = nowSeconds - noFeedbackSince
                        >= positiveOrDefault(noFeedbackTimeoutSeconds, 1.5);
            } else {
                noFeedbackSince = Double.NaN;
            }
        }

        double nextVelocity = fault ? Double.NaN : (usingLeft ? this.leftTPS : this.rightTPS);
        double dt = nowSeconds - previousTime;
        acceleration = previousSource == usingLeft && dt > 0 && dt < 0.25
                && Double.isFinite(velocity) && Double.isFinite(nextVelocity)
                ? (nextVelocity - velocity) / dt : 0;
        velocity = nextVelocity;
        previousTime = nowSeconds;
        return velocity;
    }

    public boolean isUsingLeft() { return usingLeft; }
    public boolean hasFault() { return fault; }
    public double getVelocity() { return velocity; }
    public double getAcceleration() { return acceleration; }
    public double getLeftTPS() { return leftTPS; }
    public double getRightTPS() { return rightTPS; }
    public String getSourceName() { return usingLeft ? "shooter1 (left)" : "shooter2 (right)"; }

    public void reset() {
        usingLeft = true;
        fault = false;
        suspectSince = noFeedbackSince = previousTime = Double.NaN;
        velocity = leftTPS = rightTPS = Double.NaN;
        acceleration = 0;
    }

    private static double positiveOrDefault(double value, double fallback) {
        return Double.isFinite(value) && value > 0 ? value : fallback;
    }
}
