package org.firstinspires.ftc.teamcode.bobot.subsystem;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class VisionAimTrimTest {
    private VisionAimTrim trim;

    @Before
    public void setUp() {
        VisionAimTrim.enabled = true;
        VisionAimTrim.gainPerSecond = 2;
        VisionAimTrim.maxTrimDegrees = 8;
        VisionAimTrim.maxRateDegreesPerSecond = 6;
        VisionAimTrim.maxErrorDegrees = 15;
        VisionAimTrim.deadbandDegrees = 0.3;
        trim = new VisionAimTrim();
    }

    @Test
    public void repeatedFrameCannotAccumulateCorrection() {
        trim.update(4, 1, 0);
        assertEquals(0.12, trim.update(4, 2, 0.02), 1e-9);
        assertEquals(0.12, trim.update(4, 2, 0.04), 1e-9);
        assertEquals(0.12, trim.update(4, 2, 0.08), 1e-9);
    }

    @Test
    public void correctionRemainsSmallDuringPersistentError() {
        for (int i = 0; i < 1000; i++) trim.update(10, i, i * 0.02);
        assertEquals(8, trim.getDegrees(), 1e-9);
        trim.reset();
        for (int i = 0; i < 1000; i++) trim.update(-10, i, i * 0.02);
        assertEquals(-8, trim.getDegrees(), 1e-9);
    }

    @Test
    public void smallErrorConvergesForBiasedTurret() {
        for (int i = 0; i < 200; i++) trim.update(3 - trim.getDegrees(), i, i * 0.02);
        assertTrue(Math.abs(3 - trim.getDegrees()) <= 0.3);
    }

    @Test
    public void noTargetOrGrossErrorClearsTrim() {
        trim.update(4, 1, 0);
        trim.update(4, 2, 0.02);
        assertEquals(0, trim.update(Double.NaN, 3, 0.04), 0);
        trim.update(4, 4, 0.06);
        assertEquals(0, trim.update(30, 5, 0.08), 0);
    }

    @Test
    public void disableAndResetDiscardPreviousTargetHistory() {
        trim.update(4, 1, 0);
        trim.update(4, 2, 0.02);
        VisionAimTrim.enabled = false;
        assertEquals(0, trim.update(4, 3, 0.04), 0);
        VisionAimTrim.enabled = true;
        trim.update(4, 4, 0.06);
        trim.update(4, 5, 0.08);
        trim.reset();
        assertEquals(0, trim.getDegrees(), 0);
        assertEquals(0, trim.update(4, 6, 10), 0);
    }

    @Test
    public void loopPauseCannotCauseLargeJump() {
        trim.update(4, 1, 0);
        assertEquals(0.6, trim.update(4, 2, 10), 1e-9);
    }

    @Test
    public void mechanicalLimitPreventsHiddenWindup() {
        for (int i = 0; i < 100; i++) {
            trim.update(4, i, i * 0.02);
            trim.limitTo(-1, 0.2);
        }
        assertEquals(0.2, trim.getDegrees(), 1e-9);
        assertTrue(trim.update(-4, 101, 2.02) < 0.2);
    }

    @Test
    public void invalidTuningAndTimeDoNotCommandNaN() {
        trim.update(4, 1, 0);
        VisionAimTrim.gainPerSecond = Double.NaN;
        assertEquals(0, trim.update(4, 2, 0.02), 0);
        VisionAimTrim.gainPerSecond = 2;
        assertEquals(0, trim.update(4, Double.NaN, 0.02), 0);
        assertEquals(0, trim.update(4, 3, Double.NaN), 0);
    }

    @Test
    public void fieldTargetsSelectTheirOwnCell() {
        assertEquals(Limelight.Cell.redFar, Limelight.Cell.forFieldTarget(53, 58));
        assertEquals(Limelight.Cell.redAudience, Limelight.Cell.forFieldTarget(85, 58));
        assertEquals(Limelight.Cell.blueFar, Limelight.Cell.forFieldTarget(53, 81));
        assertEquals(Limelight.Cell.blueAudience, Limelight.Cell.forFieldTarget(85, 81));
    }
}
