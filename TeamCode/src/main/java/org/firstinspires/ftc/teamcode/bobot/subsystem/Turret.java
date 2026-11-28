package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.bobot.util.AprilTagCluster;
import org.firstinspires.ftc.teamcode.bobot.util.HybridAim;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
public class Turret extends SubsystemBase {
    private static final double ANGLE_ROUNDING_TOLERANCE_DEGREES = 1e-9;
    public static double nurikGay = 395.102;
    private static final double FIELD_MIDPOINT = 72;
    private static final Pose SOUTHWEST_TARGET = new Pose(58, 60, 0);
    private static final Pose SOUTHEAST_TARGET = new Pose(86, 60, 0);
    private static final Pose NORTHWEST_TARGET = new Pose(58, 84, 0);
    private static final Pose NORTHEAST_TARGET = new Pose(86, 84, 0);

    public static double centerPosition = 0.5;
    public static double centerAngleDegrees = 180.0;
    public static double minAngleDegrees = 15;
    public static double maxAngleDegrees = 345;
    public static boolean reverseServo1 = false;
    public static boolean reverseServo2 = false;
    public static double turnLeadSeconds = 0.0;
    public static double servoDegPerSec = 400;
    public static double atTargetDeg = 1.5;
    private Limelight limelight;
    private AprilTagCluster visionCluster;
    private final HybridAim visionAim = new HybridAim();
    private double estimatedAngleDegrees;
    private long lastEstimateNanos;
    private boolean seesTags;
    private boolean outOfRange;
    private ServoEx servo1, servo2;
    private double targetAngleDegrees;
    private double aimTrimRadians;
    private boolean on = true;

    public void init(HardwareMap hw) {
        init(hw, null);
    }

    public void init(HardwareMap hw, Limelight limelight) {
        this.limelight = limelight;
        visionCluster = null;
        outOfRange = false;
        lastEstimateNanos = System.nanoTime();
        estimatedAngleDegrees = centerAngleDegrees;
        visionAim.reset(lastEstimateNanos * 1e-9);
        seesTags = false;
        servo1 = new ServoEx(hw, "servo1");
        servo2 = new ServoEx(hw, "servo2");
        targetAngleDegrees = centerAngleDegrees;
        aimTrimRadians = 0;
        servo1.setCachingTolerance(0.0001);
        servo2.setCachingTolerance(0.0001);
    }

    public static Pose selectFieldTarget(Pose pose) {
        return selectFieldTarget(pose, pose.y() >= FIELD_MIDPOINT);
    }

    public static Pose selectFieldTarget(Pose pose, boolean north) {
        boolean east = pose.x() >= FIELD_MIDPOINT;
        if (north) {
            return east ? NORTHEAST_TARGET : NORTHWEST_TARGET;
        }
        return east ? SOUTHEAST_TARGET : SOUTHWEST_TARGET;
    }

    public void face(Pose target, Pose robot) {
        face(target, robot, 0.0);
    }

    public void setVisionCluster(AprilTagCluster cluster) {
        visionCluster = cluster;
        seesTags = false;
        resetVisionOffset();
    }

    public void face(Pose target, Pose robot, double headingRateRadPerSec) {
        face(target, robot, headingRateRadPerSec, selectFieldTarget(target));
    }

    public void face(Pose target, Pose robot, double headingRateRadPerSec, Pose visionTarget) {
        double dx = target.x() - robot.x();
        double dy = target.y() - robot.y();
        if (dx == 0 && dy == 0) return;

        double predictedHeading = robot.heading() + headingRateRadPerSec * turnLeadSeconds;
        double bearing = Math.atan2(dy, dx);
        // Servo midpoint faces backward; yaw commands are relative to that direction.
        double baselineYaw = bearing - predictedHeading - Math.PI + aimTrimRadians;
        double biasDegrees = Math.toDegrees(normalizeAngle(bearing
                - Math.atan2(visionTarget.y() - robot.y(), visionTarget.x() - robot.x())
                + aimTrimRadians - headingRateRadPerSec * turnLeadSeconds));
        setYaw(baselineYaw - Math.toRadians(visionAim.getOffset()));
        boolean reachable = !outOfRange;
        double correction = visionAim.getOffset();
        if (limelight != null) {
            if (!limelight.isRunning()) limelight.start();
            limelight.periodic();
            double tx = limelight.getHiveTx(visionCluster != null ? visionCluster
                    : AprilTagCluster.forTarget(visionTarget.x(), visionTarget.y()));
            if (limelight.isNewFrame()) seesTags = !Double.isNaN(tx);
            correction = visionAim.update(tx - biasDegrees, limelight.isNewFrame(), reachable && isAtTarget(),
                    limelight.getFrameAgeMs(), System.nanoTime() * 1e-9);
        }
        setYaw(baselineYaw - Math.toRadians(correction));
    }

    public void setYaw(double radians) {
        double desired = centerAngleDegrees + Math.toDegrees(normalizeAngle(radians));
        double low = lowLimit();
        double high = highLimit();
        double shortest = Double.POSITIVE_INFINITY;
        double best = low;

        for (int turns = -2; turns <= 2; turns++) {
            double candidate = desired + turns * 360.0;
            if (!(candidate >= low - ANGLE_ROUNDING_TOLERANCE_DEGREES
                    && candidate <= high + ANGLE_ROUNDING_TOLERANCE_DEGREES)) continue;
            candidate = Math.max(low, Math.min(high, candidate));
            double distance = Math.abs(candidate - targetAngleDegrees);
            if (distance < shortest) {
                best = candidate;
                shortest = distance;
            }
        }

        outOfRange = shortest == Double.POSITIVE_INFINITY;
        if (outOfRange) {
            double toLow = Math.abs(normalizeAngle(Math.toRadians(desired - low)));
            double toHigh = Math.abs(normalizeAngle(Math.toRadians(desired - high)));
            best = toLow < toHigh ? low : high;
        }
        targetAngleDegrees = best;
    }

    public boolean isOutOfRange() { return outOfRange; }

    public double getYaw() {
        return Math.toRadians(targetAngleDegrees - centerAngleDegrees);
    }

    public void addYaw(double radians) {
        double angle = targetAngleDegrees + Math.toDegrees(radians);
        outOfRange = angle < lowLimit() || angle > highLimit();
        targetAngleDegrees = Math.max(lowLimit(), Math.min(highLimit(), angle));
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public void nudge() {
        nudge(0.3);
    }

    public void nudge(double radians) {
        // Keep the correction when face() recalculates tracking on the next loop.
        aimTrimRadians += radians;
        addYaw(radians);
    }

    public boolean isAtTarget() {
        return Math.abs(targetAngleDegrees - estimatedAngleDegrees) < atTargetDeg;
    }

    public double getVisionOffsetDegrees() { return visionAim.getOffset(); }
    public boolean seesTags() { return seesTags; }

    public void resetVisionOffset() {
        visionAim.reset(System.nanoTime() * 1e-9);
    }

    @Override
    public void periodic() {
        long now = System.nanoTime();
        double step = servoDegPerSec * Math.min((now - lastEstimateNanos) * 1e-9, 0.1);
        lastEstimateNanos = now;
        if (on) {
            estimatedAngleDegrees += Math.max(-step,
                    Math.min(step, targetAngleDegrees - estimatedAngleDegrees));
            double position = centerPosition + (targetAngleDegrees - centerAngleDegrees) * getKV();
            if (!(position >= 0.0 && position <= 1.0 && targetAngleDegrees >= minAngleDegrees && targetAngleDegrees <= maxAngleDegrees)) {
                return;
            }
            servo1.setInverted(reverseServo1);
            servo2.setInverted(reverseServo2);
            servo1.set(position);
            servo2.set(position);
        }
    }

    private static double getKV() {
        return 1.0 / nurikGay;
    }

    private static double lowLimit() {
        return Math.max(minAngleDegrees, centerAngleDegrees - centerPosition / getKV()) + 0.5;
    }

    public Command nudgeleft() { return new InstantCommand(this::nudge, this); }

    private static double highLimit() {
        return Math.min(maxAngleDegrees, centerAngleDegrees + (1.0 - centerPosition) / getKV()) - 0.5;
    }

    public static double normalizeAngle(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
