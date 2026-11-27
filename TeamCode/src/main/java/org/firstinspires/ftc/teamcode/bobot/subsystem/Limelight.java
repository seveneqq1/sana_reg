package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Limelight extends SubsystemBase {
    public static double cameraHeight = 10.986, cameraPitch = 45, splitHeight = 40;

    public static double turretForwardOffsetInches = 0;
    public static double turretLeftOffsetInches = 0;
    public static double turretUpOffsetInches = 188.0 / 25.4;

    public static double cameraForwardOffsetInches = 5.101;
    public static double cameraLeftOffsetInches = 0;
    public static double cameraUpOffsetInches = 3.584425197;

    public static double shooterForwardOffsetInches = 0;
    public static double shooterLeftOffsetInches = 0;
    public static double shooterUpOffsetInches = 0;

    public static double cameraRollDegrees = 180;
    public static double cameraYawOffsetDegrees = 0;
    public static double shooterYawOffsetDegrees = 0;

    public static int confirmLoops = 3;
    public static int pipeline = 1;
    public static double maximumFrameAgeMs = 100;

    public static volatile boolean enabled = true;
    private boolean polling;
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
        if (enabled) {
            limelight.start();
            polling = true;
        }
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
        if (!enabled) {
            if (polling) limelight.stop();
            polling = false;
            latestResult = null;
            frameTimestamp = Double.NaN;
            invalidateMeasurements();
            reset();
            return;
        }
        if (!polling) {
            limelight.pipelineSwitch(pipeline);
            limelight.start();
            polling = true;
        }

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

        double sumCorrection = 0;
        int count = 0;
        for (FiducialResult tag : result.getFiducialResults()) {
            double correction = target.getCorrectionDegrees(
                    tag.getFiducialId(), tag.getTargetXDegrees());
            if (!Double.isFinite(correction)) continue;
            sumCorrection += correction;
            count++;
        }
        if (count == 0) {
            invalidateMeasurements();
            return;
        }
        angle = sumCorrection / count;
        tagCount = count;
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
        polling = false;
        latestResult = null;
        frameTimestamp = Double.NaN;
        invalidateMeasurements();
    }

    public double getAngle() {
        return hasTarget() ? angle : Double.NaN;
    }

    public boolean hasTarget() {
        return enabled && Double.isFinite(angle) && validAge(getFrameAgeMs());
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
        telemetry.addData("Limelight enabled", enabled);
        telemetry.addData("Limelight cell", target);
        telemetry.addData("Limelight tags", getTagCount());
        telemetry.addData("Limelight minimum tags", VisionAimTrim.minimumTags);
        telemetry.addData("Limelight correction error (deg)", getAngle());
        telemetry.addData("Limelight frame age (ms)", getFrameAgeMs());
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

        double getCorrectionDegrees(int tagId, double txDegrees) {
            int index = tagId - firstId;
            if (index < 0 || index >= 4 || !Double.isFinite(txDegrees)) return Double.NaN;
            return -txDegrees;
        }

        public static Cell forFieldTarget(double x, double y) {
            if (y >= 70.5) return x >= 70.5 ? blueAudience : blueFar;
            return x >= 70.5 ? redAudience : redFar;
        }
    }
}
