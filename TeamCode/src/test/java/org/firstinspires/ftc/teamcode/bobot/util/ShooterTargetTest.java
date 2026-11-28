package org.firstinspires.ftc.teamcode.bobot.util;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.junit.Test;
import static org.junit.Assert.*;

public class ShooterTargetTest {
    @Test public void fixedTargetOverridesDefaultWithoutChangingBaseVelocity() {
        RobotNew.ShooterNew shooter = new RobotNew.ShooterNew();
        double base = RobotNew.ShooterNew.baseVel;
        assertEquals(base, shooter.getTarget(), 0);
        shooter.setTargetTPS(1450);
        assertEquals(1450, shooter.getTarget(), 0);
        assertEquals(1450, shooter.getCommandedTarget(), 0);
        assertEquals(base, RobotNew.ShooterNew.baseVel, 0);
        shooter.setTargetTPS(1500);
        assertEquals(1500, shooter.getTarget(), 0);
    }

    @Test public void invalidOrZeroTargetCannotRequestPositiveSpeed() {
        RobotNew.ShooterNew shooter = new RobotNew.ShooterNew();
        for (double target : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            shooter.setTargetTPS(target);
            assertEquals(0, shooter.getCommandedTarget(), 0);
        }
    }
}
