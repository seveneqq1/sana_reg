
package org.firstinspires.ftc.teamcode.bobot.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;

import java.util.LinkedHashMap;
import java.util.Map;

public class BekaBaseNew extends OpMode {

    protected GamepadEx first_driver, second_driver;
    protected RobotNew r;
    protected double deltaTime;

    private final ElapsedTime deltaTimer = new ElapsedTime();
    private final Map<LynxModule, LynxModule.BulkCachingMode> previousCacheModes =
            new LinkedHashMap<>();
    private boolean robotInitialized;

    protected void onInit() {}

    protected void onStart() {}

    protected void onUpdate() {}

    protected void setShooting(boolean shooting) {
        r.shooter.setStandby(false);
    }

    protected boolean isShooterReady(double velocity, double tolerance) {
        double target = r.shooter.getTarget();
        return target > 0 && Math.abs(velocity - target) < tolerance;
    }

    @Override
    public final void init() {
        first_driver = new GamepadEx(gamepad1);
        second_driver = new GamepadEx(gamepad2);

        r = RobotNew.getInstance();
        r.init(hardwareMap);
        robotInitialized = true;

        onInit();
    }

    @Override
    public final void start() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            previousCacheModes.put(hub, hub.getBulkCachingMode());
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        clearBulkCaches();
        deltaTimer.reset();

        r.shooter.setStandby(false);
        r.shooter.turnOn();

        onStart();
    }

    @Override
    public final void loop() {
        deltaTime = deltaTimer.seconds();
        deltaTimer.reset();

        clearBulkCaches();

        first_driver.readButtons();
        second_driver.readButtons();
        if (first_driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            r.flower.toggleFlower();
        }

        r.shooter.setStandby(false);

        onUpdate();
        r.periodic();
    }

    @Override
    public final void stop() {
        try {
            if (robotInitialized) {
                clearBulkCaches();
                try {
                    r.drive.periodic();
                    r.saveEnd();
                } finally {
                    r.turnOff();
                }
            }
        } finally {
            robotInitialized = false;

            for (Map.Entry<LynxModule, LynxModule.BulkCachingMode> entry
                    : previousCacheModes.entrySet()) {
                entry.getKey().setBulkCachingMode(entry.getValue());
            }

            previousCacheModes.clear();
        }
    }

    private void clearBulkCaches() {
        for (LynxModule hub : previousCacheModes.keySet()) {
            hub.clearBulkCache();
        }
    }
}
