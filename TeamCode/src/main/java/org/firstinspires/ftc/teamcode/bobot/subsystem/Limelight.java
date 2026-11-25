package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Limelight extends SubsystemBase {
    public static double cameraHeight = 12, cameraPitch = 0, splitHeight = 40;
    public static int confirmLoops = 3;
    public static int pipeline = 0;
    public static double maximumFrameAgeMs = 100;

    private Limelight3A limelight;
    private Cell target = Cell.redAudience;
    private boolean wasUp, tipped;
    private int lowLoops;
    private double angle = Double.NaN, height = Double.NaN;
    private double frameTimestamp = Double.NaN;
    private long frameChangedNanos;
    private LLResult latestResult;
    private int tagCount;

    public void init(HardwareMap hw) {
        stop();
        limelight = hw.tryGet(Limelight3A.class, "limelight");
        target = Cell.redAudience;
        reset();
        if (limelight == null) return;
        limelight.pipelineSwitch(pipeline);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void setTarget(Cell target) {
        if (target == null) throw new IllegalArgumentException("Target cell must not be null");
        if (target != this.target) {
            this.target = target;
            reset();
            invalidateMeasurements();
            latestResult = null;
        }
    }

    @Override
    public void periodic() {
        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getPipelineIndex() != pipeline
                || !Double.isFinite(result.getTimestamp()) || !validAge(result.getStaleness())) {
            invalidateMeasurements();
            return;
        }

        boolean newFrame = result.getTimestamp() != frameTimestamp;
        if (newFrame) {
            frameTimestamp = result.getTimestamp();
            frameChangedNanos = System.nanoTime();
        }
        latestResult = result;
        if (!validAge(getFrameAgeMs())) {
            invalidateMeasurements();
            return;
        }
        if (!newFrame && tagCount > 0) return;

        double sumX = 0, sumHeight = 0;
        int count = 0, heightCount = 0;

        if (result != null && result.isValid()) {
            for (FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id < target.firstId || id > target.firstId + 3) continue;
                if (!Double.isFinite(tag.getTargetXDegrees())) continue;

                sumX += tag.getTargetXDegrees();
                count++;
                if (tag.getTargetPoseCameraSpace() != null) {
                    Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
                    double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                    double tagHeight = cameraHeight + range
                            * Math.sin(Math.toRadians(cameraPitch + tag.getTargetYDegrees()));
                    if (Double.isFinite(tagHeight)) {
                        sumHeight += tagHeight;
                        heightCount++;
                    }
                }
            }
        }

        if (count == 0) {
            invalidateMeasurements();
            return;
        }

        angle = -sumX / count;
        tagCount = count;
        height = heightCount > 0 ? sumHeight / heightCount : Double.NaN;
        if (!newFrame || !Double.isFinite(height)) return;

        if (height > splitHeight) {
            wasUp = true;
            lowLoops = 0;
        } else if (wasUp && ++lowLoops >= confirmLoops) {
            tipped = true;
        }
    }

    public void reset() {
        wasUp = false;
        tipped = false;
        lowLoops = 0;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
            limelight = null;
        }
        latestResult = null;
        frameTimestamp = Double.NaN;
        invalidateMeasurements();
    }

    public double getAngle() {
        return hasTarget() ? angle : Double.NaN;
    }

    public boolean hasTarget() {
        return Double.isFinite(angle) && validAge(getFrameAgeMs());
    }

    public boolean isTipped() {
        return tipped;
    }

    public double getHeight() {
        return height;
    }

    public int getTagCount() {
        return hasTarget() ? tagCount : 0;
    }

    public double getFrameTimestamp() {
        return frameTimestamp;
    }

    public double getFrameAgeMs() {
        if (latestResult == null) return Double.POSITIVE_INFINITY;
        double latencyMs = latestResult.getCaptureLatency() + latestResult.getTargetingLatency();
        if (!Double.isFinite(latencyMs) || latencyMs < 0) return Double.POSITIVE_INFINITY;
        return Math.max(latestResult.getStaleness(), (System.nanoTime() - frameChangedNanos) * 1e-6)
                + latencyMs;
    }

    public Cell getTarget() {
        return target;
    }

    public void debug(Telemetry telemetry) {
        telemetry.addData("Limelight cell", target);
        telemetry.addData("Limelight tags", getTagCount());
        telemetry.addData("Limelight correction error (deg)", getAngle());
        telemetry.addData("Limelight frame age (ms)", getFrameAgeMs());
        telemetry.addData("Limelight height (in)", height);
        telemetry.addData("Limelight tipped", tipped);
    }

    private boolean validAge(double ageMs) {
        return Double.isFinite(maximumFrameAgeMs) && maximumFrameAgeMs > 0
                && ageMs >= 0 && ageMs <= maximumFrameAgeMs;
    }

    private void invalidateMeasurements() {
        angle = Double.NaN;
        height = Double.NaN;
        tagCount = 0;
    }

    public enum Cell {
        redFar(30), redAudience(34), blueAudience(38), blueFar(42);

        private final int firstId;

        Cell(int firstId) {
            this.firstId = firstId;
        }

        public static Cell forFieldTarget(double x, double y) {
            if (y >= 70.5) return x >= 70.5 ? blueAudience : blueFar;
            return x >= 70.5 ? redAudience : redFar;
        }
    }
}
