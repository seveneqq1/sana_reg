package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;

import static org.junit.Assert.*;

public class ShooterNewEncoderFailoverTest {
    private final Motor left = new Motor();
    private final Motor right = new Motor();
    private RobotNew.ShooterNew shooter;
    private double stopperPosition;
    private boolean previousOn, previousVolComp;
    private double previousTarget;

    @Before
    public void setUp() throws Exception {
        previousOn = RobotNew.ShooterNew.on;
        previousVolComp = RobotNew.ShooterNew.volComp;
        previousTarget = RobotNew.ShooterNew.baseVel;
        RobotNew.ShooterNew.on = true;
        RobotNew.ShooterNew.volComp = false;
        RobotNew.ShooterNew.baseVel = 1367;
        shooter = new RobotNew.ShooterNew();
        shooter.l = left.device;
        shooter.r = right.device;
        Servo stopper = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) stopperPosition = (double) args[0];
                    return null;
                });
        Field field = RobotNew.ShooterNew.class.getDeclaredField("stopper");
        field.setAccessible(true);
        field.set(shooter, stopper);
    }

    @After
    public void restoreSettings() {
        RobotNew.ShooterNew.on = previousOn;
        RobotNew.ShooterNew.volComp = previousVolComp;
        RobotNew.ShooterNew.baseVel = previousTarget;
    }

    @Test
    public void switchUsesBackupForPidAndReadinessWithoutDerivativeKick() {
        left.velocity = 20;
        right.velocity = -1367;
        shooter.set(0.7);
        shooter.encoderFeedback.update(20, -1367, 1367, 0.7, secondsAgo(1));
        shooter.periodic();
        assertFalse(shooter.encoderFeedback.isUsingLeft());
        assertEquals(1367, shooter.getVel(), 0);
        assertEquals(0, shooter.getAccel(), 0);
        assertTrue(shooter.isShooterAtTarget());
        assertEquals(RobotNew.ShooterNew.kS + RobotNew.ShooterNew.kV * 1367, left.power, 1e-9);
        assertEquals(left.power, right.power, 0);
    }

    @Test
    public void bothFailedStopBothMotorsBlockFeedingAndLatchUntilStop() {
        left.velocity = 20;
        right.velocity = 30;
        shooter.set(0.7);
        shooter.setFeeding(true);
        shooter.encoderFeedback.update(20, 30, 1367, 0.7, secondsAgo(2));
        shooter.periodic();
        assertTrue(shooter.encoderFeedback.hasFault());
        assertEquals(0, left.power, 0);
        assertEquals(0, right.power, 0);
        assertFalse(shooter.isShooterAtTarget());
        assertFalse(shooter.isShooterAlmostAtTarget());
        shooter.setFeeding(true);
        assertEquals(RobotNew.ShooterNew.blockedPosition, stopperPosition, 0);
        left.velocity = 1367;
        shooter.periodic();
        assertEquals(0, left.power, 0);
        shooter.turnOff();
        shooter.turnOn();
        shooter.periodic();
        assertFalse(shooter.encoderFeedback.hasFault());
        assertTrue(left.power > 0);
    }

    @Test
    public void zeroTargetDoesNotRunMotorsOrFaultOnStationaryEncoders() {
        RobotNew.ShooterNew.baseVel = 0;
        shooter.encoderFeedback.update(0, 0, 0, 0, secondsAgo(10));
        shooter.periodic();
        assertEquals(0, left.power, 0);
        assertEquals(0, right.power, 0);
        assertFalse(shooter.encoderFeedback.hasFault());
        assertFalse(shooter.isShooterAtTarget());
    }

    private double secondsAgo(double seconds) {
        return System.nanoTime() * 1e-9 - seconds;
    }

    private static class Motor {
        double velocity;
        double power;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getVelocity": return velocity;
                        case "setPower": power = (double) args[0]; return null;
                        case "getPower": return power;
                        default: throw new UnsupportedOperationException(method.getName());
                    }
                });
    }
}
