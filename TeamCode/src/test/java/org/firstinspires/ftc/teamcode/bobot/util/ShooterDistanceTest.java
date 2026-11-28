package org.firstinspires.ftc.teamcode.bobot.util;
import org.junit.Test;
import static org.junit.Assert.*;
public class ShooterDistanceTest {
    @Test public void matchesMeasuredLinearFit() {
        assertEquals(1049.03115, ShooterDistance.targetTPS(0), 1e-8);
        assertEquals(1633.24715, ShooterDistance.targetTPS(100), 1e-8);
        assertEquals(1925.35515, ShooterDistance.targetTPS(150), 1e-8);
    }
    @Test public void invalidDistancesDisableTarget() {
        assertEquals(0, ShooterDistance.targetTPS(-1), 0);
        assertEquals(0, ShooterDistance.targetTPS(Double.NaN), 0);
        assertEquals(0, ShooterDistance.targetTPS(Double.POSITIVE_INFINITY), 0);
    }
}
