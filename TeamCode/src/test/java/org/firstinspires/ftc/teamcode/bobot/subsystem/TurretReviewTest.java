package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.bobot.util.AprilTagCluster;
import org.junit.Test;
import java.lang.reflect.Field;
import static org.junit.Assert.*;

public class TurretReviewTest {
    private void field(Turret turret, String name, Object value) throws Exception {
        Field f = Turret.class.getDeclaredField(name);
        f.setAccessible(true);
        f.set(turret, value);
    }

    @Test public void negativeYawIsReachableAndOutsideTravelIsReported() {
        Turret turret = new Turret();
        turret.setYaw(Math.toRadians(-90));
        assertEquals(-90, Math.toDegrees(turret.getYaw()), 1e-8);
        assertFalse(turret.isOutOfRange());
        turret.setYaw(Math.PI);
        assertTrue(turret.isOutOfRange());
        assertTrue(Math.abs(Math.toDegrees(turret.getYaw())) <= 165);
    }

    @Test public void targetChangeMustNotUseOldSettledState() throws Exception {
        Turret turret = new Turret();
        turret.setYaw(0);
        field(turret, "estimatedAngleDegrees", 180.0);
        field(turret, "limelight", new Camera(10));
        turret.face(new Pose(0, 10, 0), new Pose(0, 0, 0), 0);
        assertEquals(0, turret.getVisionOffsetDegrees(), 0);
    }

    @Test public void manualAimBiasDoesNotBecomeVisionError() throws Exception {
        Turret turret = new Turret();
        double bias = Math.toRadians(10);
        turret.setYaw(bias);
        field(turret, "estimatedAngleDegrees", 190.0);
        field(turret, "limelight", new Camera(10));
        turret.face(new Pose(-10 * Math.cos(bias), -10 * Math.sin(bias), 0),
                new Pose(0, 0, 0), 0, new Pose(-10, 0, 0));
        assertEquals(0, turret.getVisionOffsetDegrees(), 1e-8);
        assertEquals(10, Math.toDegrees(turret.getYaw()), 1e-8);
    }

    @Test public void backwardTargetUsesServoMidpoint() {
        Turret turret = new Turret();
        turret.face(new Pose(0, -10, 0), new Pose(0, 0, Math.PI / 2), 0);
        assertEquals(0, Math.toDegrees(turret.getYaw()), 1e-8);
        assertFalse(turret.isOutOfRange());
    }

    @Test public void chassisRotationIsCounteracted() {
        Turret turret = new Turret();
        turret.face(new Pose(0, -10, 0), new Pose(0, 0, Math.toRadians(100)), 0);
        assertEquals(-10, Math.toDegrees(turret.getYaw()), 1e-8);
    }

    private static class Camera extends Limelight {
        private final double tx;
        Camera(double tx) { this.tx = tx; }
        @Override public boolean isRunning() { return true; }
        @Override public void periodic() {}
        @Override public double getHiveTx(AprilTagCluster cluster) { return tx; }
        @Override public boolean isNewFrame() { return true; }
        @Override public double getFrameAgeMs() { return 0; }
    }
}
