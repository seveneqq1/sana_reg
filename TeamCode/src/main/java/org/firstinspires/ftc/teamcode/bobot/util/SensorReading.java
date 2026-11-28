package org.firstinspires.ftc.teamcode.bobot.util;

import java.util.function.DoubleSupplier;

/** Shares hardware readings for 100 ms, independent of loop frequency. */
public class SensorReading {
    private boolean sampled;
    private long lastSampleNanos;
    private double value;

    public double get(DoubleSupplier sensor) {
        return getAt(System.nanoTime(), sensor);
    }

    double getAt(long now, DoubleSupplier sensor) {
        if (!sampled || now - lastSampleNanos >= 100_000_000L) {
            value = sensor.getAsDouble();
            lastSampleNanos = now;
            sampled = true;
        }
        return value;
    }

    public void reset() { sampled = false; }
}
