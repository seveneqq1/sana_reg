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
    public static double nurikGay = 422.0;
    private static final double FIELD_MIDPOINT = 70.5;
    private static final Pose SOUTHWEST_TARGET = new Pose(53, 58, 0);
    private static final Pose SOUTHEAST_TARGET = new Pose(85, 58, 0);
    private static final Pose NORTHWEST_TARGET = new Pose(53, 81, 0);
    private static final Pose NORTHEAST_TARGET = new Pose(85, 81, 0);

    public static double centerPosition = 0.062;
    public static double centerAngleDegrees = 180.0;
    public static double kV = 1.0 / nurikGay;
    public static double minAngleDegrees = -60.0;
    public static double maxAngleDegrees = 600.0;
    public static boolean reverseServo1 = false;
    public static boolean reverseServo2 = false;
    public static double turnLeadSeconds = 0.0;
    private ServoEx servo1, servo2;
    private double targetAngleDegrees;
    private double aimTrimRadians;
    private boolean on = true;

    public void init(HardwareMap hw) {
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

    public void face(Pose target, Pose robot, double headingRateRadPerSec) {
        double dx = target.x() - robot.x();
        double dy = target.y() - robot.y();
        if (dx == 0 && dy == 0) return;

        double predictedHeading = robot.heading() + headingRateRadPerSec * turnLeadSeconds;
        setYaw(Math.atan2(dy, dx) - predictedHeading + aimTrimRadians);
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

        if (shortest == Double.POSITIVE_INFINITY) {
            double toLow = Math.abs(normalizeAngle(Math.toRadians(desired - low)));
            double toHigh = Math.abs(normalizeAngle(Math.toRadians(desired - high)));
            best = toLow < toHigh ? low : high;
        }
        targetAngleDegrees = best;
    }

    public double getYaw() {
        return Math.toRadians(targetAngleDegrees - centerAngleDegrees);
    }

    public void addYaw(double radians) {
        double angle = targetAngleDegrees + Math.toDegrees(radians);
        targetAngleDegrees = Math.max(lowLimit(), Math.min(highLimit(), angle));
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    public void nudge() {
        // Keep the correction when face() recalculates tracking on the next loop.
        aimTrimRadians += 0.3;
        addYaw(0.3);
    }

    @Override
    public void periodic() {
        if (on) {
            double position = centerPosition + (targetAngleDegrees - centerAngleDegrees) * kV;
            if (!(position >= 0.0 && position <= 1.0 && targetAngleDegrees >= minAngleDegrees && targetAngleDegrees <= maxAngleDegrees)) {
                return;
            }
            servo1.setInverted(reverseServo1);
            servo2.setInverted(reverseServo2);
            servo1.set(position);
            servo2.set(position);
        }
    }

    private static double lowLimit() {
        return Math.max(minAngleDegrees, centerAngleDegrees - centerPosition / kV) + 0.5;
    }

    public Command nudgeleft() { return new InstantCommand(this::nudge, this); }

    private static double highLimit() {
        return Math.min(maxAngleDegrees, centerAngleDegrees + (1.0 - centerPosition) / kV) - 0.5;
    }

    public static double normalizeAngle(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
