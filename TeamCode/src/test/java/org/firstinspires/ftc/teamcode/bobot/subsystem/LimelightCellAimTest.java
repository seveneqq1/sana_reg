package org.firstinspires.ftc.teamcode.bobot.subsystem;

import org.junit.Test;
import static org.junit.Assert.*;

public class LimelightCellAimTest {
    @Test
    public void correctionUsesOnlyTxForEveryMatchingTag() {
        int first = 30;
        for (Limelight.Cell cell : Limelight.Cell.values()) {
            for (int id = first; id < first + 4; id++) {
                assertEquals(-4, cell.getCorrectionDegrees(id, 4), 1e-9);
            }
            assertTrue(Double.isNaN(cell.getCorrectionDegrees(first - 1, 4)));
            assertTrue(Double.isNaN(cell.getCorrectionDegrees(first + 4, 4)));
            assertTrue(Double.isNaN(cell.getCorrectionDegrees(first, Double.NaN)));
            assertTrue(Double.isNaN(cell.getCorrectionDegrees(first, Double.POSITIVE_INFINITY)));
            first += 4;
        }
    }

    @Test
    public void disabledCameraCannotExposeATarget() {
        boolean previous = Limelight.enabled;
        try {
            Limelight.enabled = false;
            Limelight camera = new Limelight();
            camera.periodic();
            assertFalse(camera.hasTarget());
            assertEquals(0, camera.getTagCount());
            assertTrue(Double.isNaN(camera.getAngle()));
        } finally {
            Limelight.enabled = previous;
        }
    }
}
