package org.firstinspires.ftc.teamcode.bobot.util;

import com.pedropathing.math.Pose;

public final class PoseStorage {
    private static Pose savedPose;

    private PoseStorage() {}

    public static void save(Pose pose) {
        if (pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading())) {
            savedPose = new Pose(pose.x(), pose.y(), pose.heading());
        }
    }

    public static Pose getOrDefault(Pose fallback) {
        return savedPose == null ? fallback : new Pose(savedPose.x(), savedPose.y(), savedPose.heading());
    }
}
