package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.bobot.util.AprilTagCluster;

@Config
public class Limelight {
    public static int pipeline = 1;
    public static double maxStalenessMs = 200;
    private Limelight3A camera;
    private LLResult result;
    private boolean running;
    private boolean newFrame;
    private boolean aprilTagDetected;
    private long lastStamp;

    public void init(HardwareMap hw) {
        stop();
        aprilTagDetected = false;
        camera = hw.tryGet(Limelight3A.class, "limelight");
        lastStamp = 0;
        newFrame = false;
        if (camera == null) return;
        camera.pipelineSwitch(pipeline);
        start();
    }

    public void periodic() {
        if (running) result = camera.getLatestResult();
        long stamp = result != null ? result.getControlHubTimeStamp() : 0;
        newFrame = result != null && stamp != lastStamp;
        lastStamp = stamp;
        if (newFrame && hasTarget() && !result.getFiducialResults().isEmpty()) aprilTagDetected = true;
    }

    public double getFrameAgeMs() {
        return result == null ? Double.POSITIVE_INFINITY
                : result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
    }

    public boolean hasTarget() {
        return result != null && result.isValid() && getFrameAgeMs() < maxStalenessMs;
    }

    public void resetAprilTagDetection() {
        aprilTagDetected = false;
    }

    /** Returns true once any AprilTag has been detected since init() or the last reset. */
    public boolean hasDetectedAprilTag() {
        return aprilTagDetected;
    }

    public double getHiveTx(AprilTagCluster cluster) {
        if (!hasTarget()) return Double.NaN;
        double sum = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (!cluster.contains(tag.getFiducialId())) continue;
            sum += tag.getTargetXDegrees();
            count++;
        }
        return count > 0 ? sum / count : Double.NaN;
    }

    public void start() {
        if (camera == null) return;
        camera.start();
        running = true;
    }

    public void stop() {
        if (camera != null) camera.stop();
        running = false;
        result = null;
        newFrame = false;
    }

    public boolean isNewFrame() { return newFrame; }
    public boolean isRunning() { return running; }
    public boolean isConnected() { return camera != null && camera.isConnected(); }
}
