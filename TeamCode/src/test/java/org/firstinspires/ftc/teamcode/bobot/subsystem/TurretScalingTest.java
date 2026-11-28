package org.firstinspires.ftc.teamcode.bobot.subsystem;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TurretScalingTest {
    @Test public void calibrationChangesUpdateTravelWithoutReinitializingTurret() {
        double originalScale = Turret.nurikGay;
        try {
            Turret.nurikGay = 330;
            Turret turret = new Turret();
            turret.setYaw(Math.toRadians(120));
            assertEquals(120, Math.toDegrees(turret.getYaw()), 1e-8);
            assertFalse(turret.isOutOfRange());

            Turret.nurikGay = 200;
            turret.setYaw(Math.toRadians(120));
            assertEquals(99.5, Math.toDegrees(turret.getYaw()), 1e-8);
            assertTrue(turret.isOutOfRange());
            turret.setYaw(Math.toRadians(-120));
            assertEquals(-99.5, Math.toDegrees(turret.getYaw()), 1e-8);
            assertTrue(turret.isOutOfRange());
        } finally {
            Turret.nurikGay = originalScale;
        }
    }
}
