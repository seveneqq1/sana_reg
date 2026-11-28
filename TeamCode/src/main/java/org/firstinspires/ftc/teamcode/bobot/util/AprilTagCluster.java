package org.firstinspires.ftc.teamcode.bobot.util;


public enum AprilTagCluster {
    RED_AUDIENCE(34), RED(30), BLUE_AUDIENCE(38), BLUE(42),
    RED_ALLIANCE(30, 8), BLUE_ALLIANCE(38, 8);

    private final int firstId;

    private final int count;

    AprilTagCluster(int firstId) { this(firstId, 4); }

    AprilTagCluster(int firstId, int count) {
        this.firstId = firstId;
        this.count = count;
    }

    public boolean contains(int id) { return id >= firstId && id < firstId + count; }

    public static AprilTagCluster forTarget(double x, double y) {
        if (x < 70.5) return y < 70.5 ? RED_AUDIENCE : RED;
        return y < 70.5 ? BLUE_AUDIENCE : BLUE;
    }
}
