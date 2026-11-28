package org.firstinspires.ftc.teamcode.bobot.util;

import org.junit.Test;
import static org.junit.Assert.*;

public class HybridAimTest {
    @Test public void learnsOnlyFromNewSettledFramesAndHoldsWhenLost() {
        HybridAim aim = new HybridAim();
        assertEquals(0, aim.update(10, true, false, 50, 0), 0);
        assertEquals(0, aim.update(10, true, true, 50, 0.04), 0);
        assertEquals(1.5, aim.update(10, true, true, 50, 0.06), 1e-9);
        assertEquals(1.5, aim.update(10, false, true, 50, 1), 1e-9);
        assertEquals(1.5, aim.update(Double.NaN, true, true, 50, 2), 1e-9);
        aim.reset(3);
        assertEquals(0, aim.getOffset(), 0);
    }

    @Test public void deadbandAndLimitsMatchReference() {
        HybridAim aim = new HybridAim();
        assertEquals(0, aim.update(0.75, true, true, 0, 1), 0);
        assertEquals(25, aim.update(200, true, true, 0, 2), 0);
        assertEquals(-25, aim.update(-400, true, true, 0, 3), 0);
    }

    @Test public void clustersMatchReferenceHiveCoordinates() {
        assertTrue(AprilTagCluster.forTarget(53, 58).contains(34));
        assertTrue(AprilTagCluster.forTarget(53, 81).contains(30));
        assertTrue(AprilTagCluster.forTarget(85, 58).contains(38));
        assertTrue(AprilTagCluster.forTarget(85, 81).contains(45));
        assertFalse(AprilTagCluster.forTarget(53, 58).contains(30));
    }
}
