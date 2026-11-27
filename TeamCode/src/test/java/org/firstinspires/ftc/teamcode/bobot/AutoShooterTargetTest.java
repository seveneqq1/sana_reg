package org.firstinspires.ftc.teamcode.bobot;

import org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands;
import org.firstinspires.ftc.teamcode.bobot.util.RegShooter;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class AutoShooterTargetTest {
    private final double savedBaseVel = RobotNew.ShooterNew.baseVel;
    private final boolean savedCalibrated = RegShooter.calibrated;
    private final boolean savedUseQuadratic = RegShooter.useQuadratic;

    @Before
    public void enableDistanceEquation() {
        RegShooter.calibrated = true;
        RegShooter.useQuadratic = true;
    }

    @After
    public void restoreConfiguration() {
        RobotNew.getInstance().shooter.clearTargetTPS();
        RobotNew.ShooterNew.baseVel = savedBaseVel;
        RegShooter.calibrated = savedCalibrated;
        RegShooter.useQuadratic = savedUseQuadratic;
    }

    @Test
    public void followsLatestDistanceWithoutOverwritingFixedTarget() {
        RobotNew.ShooterNew shooter = new RobotNew.ShooterNew();
        double[] distance = {60};
        RobotNew.ShooterNew.baseVel = 1234;
        shooter.setDistanceTarget(() -> distance[0]);
        assertEquals(1304.39453, shooter.getTarget(), 1e-6);
        distance[0] = 90;
        assertEquals(1884.92027, shooter.getCommandedTarget(), 1e-6);
        assertEquals(1234, RobotNew.ShooterNew.baseVel, 0);
        distance[0] = Double.NaN;
        assertEquals(0, shooter.getTarget(), 0);
    }

    @Test
    public void clearingDistanceTargetRestoresFixedSpeed() {
        RobotNew.ShooterNew shooter = new RobotNew.ShooterNew();
        RobotNew.ShooterNew.baseVel = 1234;
        shooter.setDistanceTarget(() -> 90);
        shooter.clearTargetTPS();
        assertEquals(1234, shooter.getTarget(), 0);
    }

    @Test
    public void explicitTargetReplacesDistanceTarget() {
        RobotNew.ShooterNew shooter = new RobotNew.ShooterNew();
        shooter.setDistanceTarget(() -> 90);
        shooter.setTargetTPS(1600);
        assertEquals(1600, shooter.getTarget(), 0);
    }

    @Test
    public void finalShotCommandReplacesDistanceTargetWith1850TPS() {
        RobotNew robot = RobotNew.getInstance();
        robot.shooter.setDistanceTarget(() -> 90);
        RobotNewCommands.setBaseVel(robot, 1850).initialize();
        assertEquals(1850, robot.shooter.getTarget(), 0);
    }
}
