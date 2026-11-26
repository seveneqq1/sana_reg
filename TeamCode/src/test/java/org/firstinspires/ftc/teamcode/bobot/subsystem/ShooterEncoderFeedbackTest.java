package org.firstinspires.ftc.teamcode.bobot.subsystem;

import org.junit.Test;

import static org.junit.Assert.*;

public class ShooterEncoderFeedbackTest {
    private final ShooterEncoderFeedback feedback = new ShooterEncoderFeedback();

    @Test
    public void healthyEncodersKeepLeftAndNormalizeDirection() {
        assertEquals(1300, sample(-1300, 1320, 0), 0);
        assertTrue(feedback.isUsingLeft());
    }

    @Test
    public void lowLeftSwitchesOnlyAfterPersistentFailure() {
        assertEquals(40, sample(40, 1300, 0), 0);
        assertEquals(20, sample(20, 1300, 0.20), 0);
        assertEquals(1300, sample(30, 1300, 0.26), 0);
        assertFalse(feedback.isUsingLeft());
        assertEquals(1300, feedback.getVelocity(), 0);
        assertEquals(0, feedback.getAcceleration(), 0);
    }

    @Test
    public void momentaryDropoutDoesNotSwitchAndRestartsDelay() {
        sample(0, 1300, 0);
        sample(1300, 1300, 0.2);
        sample(0, 1300, 0.3);
        sample(0, 1300, 0.4);
        assertTrue(feedback.isUsingLeft());
        sample(0, 1300, 0.56);
        assertFalse(feedback.isUsingLeft());
    }

    @Test
    public void noSwitchBackJustBecauseLeftRecovers() {
        sample(0, 1300, 0);
        sample(0, 1300, 0.3);
        sample(1300, 1310, 0.4);
        assertFalse(feedback.isUsingLeft());
        assertEquals(1310, feedback.getVelocity(), 0);
    }

    @Test
    public void canFailBackWhenRightLaterFails() {
        sample(0, 1300, 0);
        sample(0, 1300, 0.3);
        sample(1250, 0, 0.4);
        assertEquals(1250, sample(1250, 0, 0.7), 0);
        assertTrue(feedback.isUsingLeft());
    }

    @Test
    public void startupAndBriefSharedSpeedDipDoNotFault() {
        sample(0, 0, 0);
        sample(40, 40, 1);
        assertFalse(feedback.hasFault());
        sample(1300, 1300, 1.1);
        sample(0, 0, 2);
        sample(1300, 1300, 2.3);
        assertFalse(feedback.hasFault());
    }

    @Test
    public void bothLowLatchFaultUntilReset() {
        sample(0, 20, 0);
        assertTrue(Double.isNaN(sample(30, 40, 1.6)));
        assertTrue(feedback.hasFault());
        assertTrue(Double.isNaN(sample(1300, 1300, 2)));
        feedback.reset();
        assertFalse(feedback.hasFault());
        assertTrue(feedback.isUsingLeft());
        assertTrue(Double.isNaN(feedback.getVelocity()));
        assertEquals(1300, sample(1300, 1300, 3), 0);
    }

    @Test
    public void poweredTimerDoesNotRunWhileOffOrAtLowPower() {
        feedback.update(0, 0, 0, 1, 0);
        feedback.update(0, 0, 0, 1, 10);
        feedback.update(0, 0, 1367, 0, 11);
        feedback.update(0, 0, 1367, 0.1, 20);
        assertFalse(feedback.hasFault());
        sample(0, 0, 21);
        sample(0, 0, 22);
        assertFalse(feedback.hasFault());
        sample(0, 0, 22.6);
        assertTrue(feedback.hasFault());
    }

    @Test
    public void switchingRequiresPoweredShooterForFiniteDropouts() {
        feedback.update(0, 1300, 0, 0, 0);
        feedback.update(0, 1300, 0, 0, 10);
        assertTrue(feedback.isUsingLeft());
    }

    @Test
    public void nonFiniteActiveReadingImmediatelyUsesFiniteBackup() {
        assertEquals(1300, sample(Double.NaN, -1300, 0), 0);
        assertFalse(feedback.isUsingLeft());
        assertEquals(1200, sample(1200, Double.POSITIVE_INFINITY, 0.1), 0);
        assertTrue(feedback.isUsingLeft());
    }

    @Test
    public void invalidBackupDoesNotReplaceHealthyReading() {
        assertEquals(1300, sample(1300, Double.NaN, 0), 0);
        assertEquals(1300, sample(1300, Double.NaN, 10), 0);
        assertFalse(feedback.hasFault());
    }

    @Test
    public void bothNonFiniteFaultImmediatelyWhenEnabled() {
        assertTrue(Double.isNaN(sample(Double.NaN, Double.NEGATIVE_INFINITY, 0)));
        assertTrue(feedback.hasFault());
    }

    @Test
    public void slowTargetsUseProportionalDetectionThreshold() {
        feedback.update(50, 50, 50, 0.4, 0);
        assertEquals(50, feedback.update(50, 50, 50, 0.4, 3), 0);
        assertFalse(feedback.hasFault());
    }

    @Test
    public void accelerationUsesSameSourceAndSuppressesSwitchJump() {
        sample(1000, 1200, 0);
        sample(1100, 1200, 0.1);
        assertEquals(1000, feedback.getAcceleration(), 1e-6);
        sample(Double.NaN, 1200, 0.2);
        assertEquals(0, feedback.getAcceleration(), 0);
        sample(0, 1250, 0.3);
        assertEquals(500, feedback.getAcceleration(), 1e-6);
    }

    private double sample(double left, double right, double now) {
        return feedback.update(left, right, 1367, 0.7, now);
    }
}
