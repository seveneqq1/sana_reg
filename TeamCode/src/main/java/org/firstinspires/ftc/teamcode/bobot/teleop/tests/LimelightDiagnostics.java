package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Limelight;

@TeleOp(name = "Limelight diagnostics", group = "Tests")
public class LimelightDiagnostics extends OpMode {
    private Limelight3A camera;
    private LLStatus status;
    private int pipeline;
    private int pollRateHz = 100;
    private boolean readStatus = true;
    private boolean masterEnabledLastLoop = true;
    private boolean previousA, previousB, previousX, previousLeft, previousRight;
    private boolean pipelineSwitchOk;
    private String error = "none";
    private long lastLoopNanos, lastTelemetryNanos, lastStatusNanos, rateStartNanos;
    private long lastFrameChangeNanos;
    private double lastFrameTimestamp = Double.NaN;
    private int frames;
    private double observedFrameHz;
    private double periodMs, maxPeriodMs, resultReadMs, statusReadMs, telemetryMs;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(100);
        masterEnabledLastLoop = Limelight.enabled;
        pipeline = Math.max(0, Math.min(9, Limelight.pipeline));
        try {
            camera = hardwareMap.tryGet(Limelight3A.class, "limelight");
            if (camera != null) {
                camera.setPollRateHz(pollRateHz);
                pipelineSwitchOk = camera.pipelineSwitch(pipeline);
                if (Limelight.enabled) camera.start();
            }
        } catch (RuntimeException e) {
            error = e.toString();
        }
        rateStartNanos = System.nanoTime();
        telemetry.addLine("Camera only: no motors or servos are initialized.");
        telemetry.addData("Configured as limelight", camera != null);
        telemetry.addData("Init error", error);
        telemetry.addLine("Live diagnostics run during INIT and PLAY.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updateDiagnostics();
    }

    @Override
    public void start() {
        lastLoopNanos = 0;
        maxPeriodMs = 0;
    }

    @Override
    public void loop() {
        updateDiagnostics();
    }

    private void updateDiagnostics() {
        long now = System.nanoTime();
        periodMs = lastLoopNanos == 0 ? 0 : (now - lastLoopNanos) * 1e-6;
        lastLoopNanos = now;
        maxPeriodMs = Math.max(maxPeriodMs, periodMs);

        if (camera == null) {
            if (now - lastTelemetryNanos >= 100_000_000L) {
                lastTelemetryNanos = now;
                telemetry.addData("Configured as limelight", false);
                telemetry.addLine("Add a Limelight 3A named limelight to the active robot config.");
                telemetry.addLine("Check its Control Hub USB data connection and power, then re-INIT.");
                telemetry.addData("Error", error);
                telemetry.update();
            }
            return;
        }

        if (!Limelight.enabled) {
            masterEnabledLastLoop = false;
            if (camera.isRunning()) camera.stop();
            status = null;
            resetFrameRate(now);
            telemetry.addData("Limelight enabled", false);
            telemetry.update();
            return;
        }
        if (!masterEnabledLastLoop) {
            camera.start();
            masterEnabledLastLoop = true;
        }

        try {
            if (gamepad1.a && !previousA) {
                pollRateHz = pollRateHz == 100 ? 20 : 100;
                boolean running = camera.isRunning();
                // Restart the SDK scheduler so the new poll interval actually takes effect.
                camera.stop();
                camera.setPollRateHz(pollRateHz);
                if (running) camera.start();
                resetFrameRate(now);
            }
            if (gamepad1.b && !previousB) {
                if (camera.isRunning()) camera.pause();
                else camera.start();
                resetFrameRate(now);
            }
            if (gamepad1.x && !previousX) readStatus = !readStatus;
            int change = (gamepad1.dpad_right && !previousRight ? 1 : 0)
                    - (gamepad1.dpad_left && !previousLeft ? 1 : 0);
            if (change != 0) {
                pipeline = Math.max(0, Math.min(9, pipeline + change));
                pipelineSwitchOk = camera.pipelineSwitch(pipeline);
                resetFrameRate(now);
            }
        } catch (RuntimeException e) {
            error = e.toString();
        }
        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousLeft = gamepad1.dpad_left;
        previousRight = gamepad1.dpad_right;

        long readStart = System.nanoTime();
        LLResult result = camera.getLatestResult();
        resultReadMs = (System.nanoTime() - readStart) * 1e-6;
        boolean connected = camera.isConnected();
        if (camera.isRunning() && connected && result != null
                && Double.isFinite(result.getTimestamp())
                && result.getTimestamp() != lastFrameTimestamp) {
            lastFrameTimestamp = result.getTimestamp();
            lastFrameChangeNanos = System.nanoTime();
            frames++;
        }
        double rateSeconds = (now - rateStartNanos) * 1e-9;
        if (rateSeconds >= 1) {
            observedFrameHz = frames / rateSeconds;
            frames = 0;
            rateStartNanos = now;
        }

        // getStatus() performs HTTP I/O. Sample slowly and expose its cost separately.
        if (readStatus && connected && now - lastStatusNanos >= 1_000_000_000L) {
            lastStatusNanos = now;
            long statusStart = System.nanoTime();
            try {
                status = camera.getStatus();
            } catch (RuntimeException e) {
                error = e.toString();
            }
            statusReadMs = (System.nanoTime() - statusStart) * 1e-6;
        }

        if (now - lastTelemetryNanos < 100_000_000L) return;
        lastTelemetryNanos = now;
        long telemetryStart = System.nanoTime();
        telemetry.addLine("A: 20/100 Hz | B: pause/resume | X: status HTTP on/off");
        telemetry.addLine("D-pad left/right: pipeline 0-9");
        telemetry.addData("Configured as limelight", true);
        telemetry.addData("Connected (SDK recent data)", connected);
        telemetry.addData("Polling enabled / requested Hz", "%s / %d", camera.isRunning(), pollRateHz);
        telemetry.addData("Last SDK update (ms ago)", camera.getTimeSinceLastUpdate());
        telemetry.addData("Observed new frames/sec", "%.1f", observedFrameHz);
        telemetry.addData("Last frame change (ms ago)", lastFrameChangeNanos == 0
                ? "none yet" : String.format(java.util.Locale.US, "%.1f",
                (System.nanoTime() - lastFrameChangeNanos) * 1e-6));
        telemetry.addData("Requested pipeline / switch accepted", "%d / %s", pipeline, pipelineSwitchOk);
        telemetry.addData("Status HTTP enabled", readStatus);
        if (status != null) {
            telemetry.addData("Status snapshot age (ms)", "%.0f", (now - lastStatusNanos) * 1e-6);
            telemetry.addData("Device name", status.getName());
            telemetry.addData("Status pipeline / type", "%d / %s", status.getPipelineIndex(), status.getPipelineType());
            telemetry.addData("Camera FPS / CPU % / temp C", "%.1f / %.1f / %.1f",
                    status.getFps(), status.getCpu(), status.getTemp());
        }
        if (result != null) {
            double latencyMs = result.getCaptureLatency() + result.getTargetingLatency();
            double frameAgeMs = lastFrameChangeNanos == 0 ? Double.POSITIVE_INFINITY
                    : Math.max(result.getStaleness(),
                    (System.nanoTime() - lastFrameChangeNanos) * 1e-6) + latencyMs;
            boolean fresh = connected && camera.isRunning() && Double.isFinite(frameAgeMs)
                    && latencyMs >= 0 && frameAgeMs >= 0
                    && frameAgeMs <= Limelight.maximumFrameAgeMs;
            telemetry.addData("Frame age / allowed (ms)", "%.1f / %.1f",
                    frameAgeMs, Limelight.maximumFrameAgeMs);
            telemetry.addData("Fresh result (robot age limit)", fresh);
            telemetry.addData("Target valid (cached result)", result.isValid());
            telemetry.addData("Result pipeline / type", "%d / %s", result.getPipelineIndex(), result.getPipelineType());
            telemetry.addData("Result staleness (ms)", result.getStaleness());
            telemetry.addData("Camera timestamp", result.getTimestamp());
            telemetry.addData("tx (deg)", "%.2f", result.getTx());
            telemetry.addData("Target area (%)", "%.3f", result.getTa());
            telemetry.addData("Capture / targeting / parse (ms)", "%.2f / %.2f / %.2f",
                    result.getCaptureLatency(), result.getTargetingLatency(), result.getParseLatency());
            telemetry.addData("AprilTags detected", result.getFiducialResults().size());
            int displayed = 0;
            for (FiducialResult tag : result.getFiducialResults()) {
                if (displayed++ >= 8) break;
                telemetry.addData("Tag " + displayed, "ID %d: tx %.2f deg",
                        tag.getFiducialId(), tag.getTargetXDegrees());
            }
            telemetry.addLine(!connected ? "No recent camera data: check USB, power and config."
                    : !fresh ? "Frames stale or polling paused; cached target values may be old."
                    : !result.isValid() ? "Camera communicating; no valid target in this pipeline."
                    : result.getPipelineIndex() != pipeline ? "Waiting for requested pipeline."
                    : "Fresh target received. Check tag IDs against your selected robot cell.");
        }
        telemetry.addData("Loop period / peak (ms)", "%.2f / %.2f", periodMs, maxPeriodMs);
        telemetry.addData("Cached result read (ms)", "%.3f", resultReadMs);
        telemetry.addData("Last status HTTP (ms)", "%.2f", statusReadMs);
        telemetry.addData("Previous telemetry update (ms)", "%.2f", telemetryMs);
        telemetry.addData("Error (last)", error);
        telemetry.update();
        telemetryMs = (System.nanoTime() - telemetryStart) * 1e-6;
    }

    private void resetFrameRate(long now) {
        frames = 0;
        observedFrameHz = 0;
        rateStartNanos = now;
    }

    @Override
    public void stop() {
        if (camera != null) camera.stop();
    }
}
