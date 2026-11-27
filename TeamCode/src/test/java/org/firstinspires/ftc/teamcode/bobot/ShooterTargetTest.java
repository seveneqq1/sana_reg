package org.firstinspires.ftc.teamcode.bobot;

import org.junit.After;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class ShooterTargetTest {
    private final double savedBaseVel = RobotNew.ShooterNew.baseVel;

    @After
    public void restoreConfiguration() {
        RobotNew.ShooterNew.baseVel = savedBaseVel;
    }

    @Test
    public void temporaryTargetPreservesFixedSettingAndClearsBackToIt() {
        RobotNew.ShooterNew shooter = new RobotNew.ShooterNew();
        RobotNew.ShooterNew.baseVel = 1234;
        assertEquals(1234, shooter.getTarget(), 0);
        shooter.setTargetTPS(1500);
        assertEquals(1500, shooter.getTarget(), 0);
        assertEquals(1234, RobotNew.ShooterNew.baseVel, 0);
        RobotNew.ShooterNew.baseVel = 1300;
        shooter.clearTargetTPS();
        assertEquals(1300, shooter.getTarget(), 0);
    }

    @Test
    public void invalidOverrideStopsInsteadOfFallingBackToFixedSpeed() {
        RobotNew.ShooterNew shooter = new RobotNew.ShooterNew();
        RobotNew.ShooterNew.baseVel = 1234;
        for (double target : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            shooter.setTargetTPS(target);
            assertEquals(0, shooter.getTarget(), 0);
        }
    }
}
