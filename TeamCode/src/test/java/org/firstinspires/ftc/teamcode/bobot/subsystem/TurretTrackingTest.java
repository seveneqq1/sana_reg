package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.pedropathing.math.Pose;
import org.junit.Test;
import static org.junit.Assert.*;

public class TurretTrackingTest {
    @Test
    public void tracksFixedPointAndCounterRotatesWithRobot() {
        Turret turret = new Turret();
        turret.setYaw(0);
        Pose target = new Pose(110, 63, 0);
        double bearing = Math.atan2(54, -25);
        turret.face(target, new Pose(135, 9, Math.PI / 2), 0, target);
        assertEquals(bearing - Math.PI / 2, turret.getYaw(), 1e-9);
        turret.face(target, new Pose(135, 9, Math.PI / 2 + 0.2), 0, target);
        assertEquals(bearing - Math.PI / 2 - 0.2, turret.getYaw(), 1e-9);
    }

    @Test
    public void choosesNearbyEquivalentAngleAcrossWrap() {
        Turret turret = new Turret();
        turret.setYaw(Math.toRadians(179));
        turret.setYaw(Math.toRadians(-179));
        assertEquals(181, Math.toDegrees(turret.getYaw()), 1e-9);
        assertFalse(turret.isOutOfRange());
    }

    @Test
    public void waitsAtNearestLimitWhenTargetCannotBeReached() {
        double oldMaximum = Turret.maxAngleDegrees;
        try {
            Turret.maxAngleDegrees = 270;
            Turret turret = new Turret();
            turret.setYaw(Math.PI / 2);
            assertTrue(turret.isOutOfRange());
            assertEquals(89.5, Math.toDegrees(turret.getYaw()), 1e-9);
            turret.addYaw(Math.toRadians(-10));
            assertFalse(turret.isOutOfRange());
        } finally {
            Turret.maxAngleDegrees = oldMaximum;
        }
    }

    @Test
    public void resetRemovesPersistentAimTrim() {
        Turret turret = new Turret();
        Pose robot = new Pose(135, 9, Math.PI / 2);
        Pose target = new Pose(110, 63, 0);
        turret.face(target, robot, 0, target);
        double original = turret.getYaw();
        turret.nudge(0.1);
        turret.face(target, robot, 0, target);
        assertEquals(original + 0.1, turret.getYaw(), 1e-9);
        turret.resetAimTrim();
        turret.face(target, robot, 0, target);
        assertEquals(original, turret.getYaw(), 1e-9);
    }
}
