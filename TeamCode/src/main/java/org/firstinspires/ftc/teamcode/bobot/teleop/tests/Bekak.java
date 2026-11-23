package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Flower;

@TeleOp(name = "flower servo check")
public class Bekak extends OpMode {
    private final Flower flower = new Flower();
    private GamepadEx firstDriver;

    @Override
    public void init() {
        firstDriver = new GamepadEx(gamepad1);
        flower.init(hardwareMap);
    }

    @Override
    public void loop() {
        firstDriver.readButtons();
        if (firstDriver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            flower.toggleFlower();
        }
    }
}
