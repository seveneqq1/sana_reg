package org.firstinspires.ftc.teamcode.bobot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;

import static org.junit.Assert.assertEquals;

public class BatteryVoltageSamplingTest {
    @Test
    public void telemetryAndControlShareVoltageForFiveLoops() throws Exception {
        RobotNew.ShooterNew shooter = new RobotNew.ShooterNew();
        int[] reads = {0};
        VoltageSensor sensor = (VoltageSensor) Proxy.newProxyInstance(
                VoltageSensor.class.getClassLoader(), new Class<?>[]{VoltageSensor.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("getVoltage")) return 12.0 - ++reads[0];
                    throw new UnsupportedOperationException(method.getName());
                });
        Field field = RobotNew.ShooterNew.class.getDeclaredField("voltageSensor");
        field.setAccessible(true);
        field.set(shooter, sensor);
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("getVelocity")) return 0.0;
                    if (method.getName().equals("setPower")) return null;
                    throw new UnsupportedOperationException(method.getName());
                });
        shooter.l = motor;
        shooter.r = motor;
        shooter.setTargetTPS(0);

        assertEquals(11.0, shooter.getVoltage(), 0);
        for (int loop = 0; loop < 4; loop++) {
            shooter.periodic();
            assertEquals(11.0, shooter.getVoltage(), 0);
            assertEquals(11.0, shooter.getVoltage(), 0);
        }
        assertEquals(1, reads[0]);
        shooter.periodic();
        assertEquals(10.0, shooter.getVoltage(), 0);
        assertEquals(10.0, shooter.getVoltage(), 0);
        assertEquals(2, reads[0]);
    }
}
