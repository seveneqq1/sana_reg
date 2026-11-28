package org.firstinspires.ftc.teamcode.bobot.util;
import org.junit.Test;
import static org.junit.Assert.*;
public class SensorReadingTest {
    @Test public void samplesImmediatelyThenOnlyEveryHundredMilliseconds() {
        SensorReading reading = new SensorReading();
        int[] calls = {0};
        assertEquals(1, reading.getAt(0, () -> ++calls[0]), 0);
        assertEquals(1, reading.getAt(99_999_999L, () -> ++calls[0]), 0);
        assertEquals(2, reading.getAt(100_000_000L, () -> ++calls[0]), 0);
        reading.reset();
        assertEquals(3, reading.getAt(100_000_001L, () -> ++calls[0]), 0);
    }
}
