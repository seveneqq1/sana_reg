package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
public class Turret extends SubsystemBase {
    private static final double ANGLE_ROUNDING_TOLERANCE_DEGREES = 1e-9;
    public static double nurikGay = 380;
    private static final double FIELD_MIDPOINT = 70.5;
    private static final Pose SOUTHWEST_TARGET = new Pose(58.8, 58.8, 0);
    private static final Pose SOUTHEAST_TARGET = new Pose(86.5, 58.8, 0);
    private static final Pose NORTHWEST_TARGET = new Pose(58.8, 84.6, 0);
    private static final Pose NORTHEAST_TARGET = new Pose(86.5, 84.6, 0);

    public static double centerPosition = 0.055;
    public static double centerAngleDegrees = 180.0;
    public static double minAngleDegrees = -60;
    public static double maxAngleDegrees = 600;
    public static boolean reverseServo1 = false;
    public static boolean reverseServo2 = false;
    public static double turnLeadSeconds = 0.0;
    private ServoEx servo1, servo2;
    private double targetAngleDegrees;
    private double aimTrimRadians;
    private boolean on = true;
    private Limelight limelight;
    private final VisionAimTrim visionTrim = new VisionAimTrim();
    private boolean automaticAiming;
    private boolean outOfRange;
    private double visionAimBiasDegrees;

    public void init(HardwareMap hw) {
        init(hw, null);
    }

    public void init(HardwareMap hw, Limelight limelight) {
        this.limelight = limelight;
        servo1 = new ServoEx(hw, "servo1");
        servo2 = new ServoEx(hw, "servo2");
        targetAngleDegrees = centerAngleDegrees;
        outOfRange = false;
        aimTrimRadians = 0;
        clearVisionCorrection();
        servo1.setCachingTolerance(0.0001);
        servo2.setCachingTolerance(0.0001);
    }

    public static Pose selectFieldTarget(Pose pose) {
        return selectFieldTarget(pose, pose.y() >= FIELD_MIDPOINT);
    }

    public static Pose selectWestFieldTarget(Pose pose) {
        return pose.y() >= FIELD_MIDPOINT ? NORTHWEST_TARGET : SOUTHWEST_TARGET;
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

    public void face(Pose target, Pose robot, double headingRateRadPerSec) {
        face(target, robot, headingRateRadPerSec, selectFieldTarget(target));
    }

    public void face(Pose target, Pose robot, double headingRateRadPerSec, Pose visionTarget) {
        double dx = target.x() - robot.x();
        double dy = target.y() - robot.y();
        if (dx == 0 && dy == 0) {
            outOfRange = false;
            clearVisionCorrection();
            return;
        }

        double predictedHeading = robot.heading() + headingRateRadPerSec * turnLeadSeconds;
        double targetBearing = Math.atan2(dy, dx);
        double visionBearing = Math.atan2(visionTarget.y() - robot.y(), visionTarget.x() - robot.x());
        visionAimBiasDegrees = Math.toDegrees(normalizeAngle(targetBearing - visionBearing
                + aimTrimRadians - headingRateRadPerSec * turnLeadSeconds));
        if (limelight != null) {
            Limelight.Cell cell = Limelight.Cell.forFieldTarget(visionTarget.x(), visionTarget.y());
            if (cell != limelight.getTarget()) visionTrim.reset();
            limelight.setTarget(cell);
        }
        automaticAiming = true;
        setTargetYaw(targetBearing - predictedHeading + aimTrimRadians);
    }

    public void setYaw(double radians) {
        clearVisionCorrection();
        setTargetYaw(radians);
    }

    private void setTargetYaw(double radians) {
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

    public boolean isOutOfRange() {
        return outOfRange;
    }

    public void resetAimTrim() {
        aimTrimRadians = 0;
        clearVisionCorrection();
    }

    public double getYaw() {
        return Math.toRadians(targetAngleDegrees + visionTrim.getDegrees() - centerAngleDegrees);
    }

    public void addYaw(double radians) {
        outOfRange = false;
        double angle = targetAngleDegrees + visionTrim.getDegrees() + Math.toDegrees(radians);
        clearVisionCorrection();
        targetAngleDegrees = Math.max(lowLimit(), Math.min(highLimit(), angle));
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
        clearVisionCorrection();
    }

    public void nudge() {
        nudge(0.3);
    }

    public void nudge(double radians) {
        aimTrimRadians += radians;
        visionAimBiasDegrees += Math.toDegrees(radians);
        double angle = targetAngleDegrees + Math.toDegrees(radians);
        targetAngleDegrees = Math.max(lowLimit(), Math.min(highLimit(), angle));
    }

    @Override
    public void periodic() {
        if (on) {
            if (automaticAiming && limelight != null && limelight.hasTarget()
                    && limelight.getTagCount() >= Math.max(1, VisionAimTrim.minimumTags)) {
                visionTrim.update(limelight.getAngle() + visionAimBiasDegrees + VisionAimTrim.aimOffsetDegrees,
                        limelight.getFrameTimestamp(), System.nanoTime() * 1e-9);
                visionTrim.limitTo(lowLimit() - targetAngleDegrees, highLimit() - targetAngleDegrees);
            } else {
                visionTrim.reset();
            }
            double correctedAngle = targetAngleDegrees + visionTrim.getDegrees();
            double position = centerPosition + (correctedAngle - centerAngleDegrees) * getKV();
            if (!(position >= 0.0 && position <= 1.0 && correctedAngle >= minAngleDegrees && correctedAngle <= maxAngleDegrees)) {
                return;
            }
            servo1.setInverted(reverseServo1);
            servo2.setInverted(reverseServo2);
            servo1.set(position);
            servo2.set(position);
        }
    }

    public double getVisionTrimDegrees() {
        return visionTrim.getDegrees();
    }

    private void clearVisionCorrection() {
        automaticAiming = false;
        visionTrim.reset();
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
