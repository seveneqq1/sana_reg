package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Flower;

@Disabled
@TeleOp(name = "bekassss")
public class turretS extends OpMode {
    private final Flower flower = new Flower();
    Servo servo1, servo2;

    private GamepadEx firstDriver;

    @Override
    public void init() {
        firstDriver = new GamepadEx(gamepad1);
        flower.init(hardwareMap);
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void loop() {
        firstDriver.readButtons();
        if (firstDriver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            flower.toggleFlower();
        }
        if(gamepad1.left_trigger_pressed){
            servo1.setPosition(0);
        }
        else{
            servo1.setPosition(0.5);
        }
        if(gamepad1.right_trigger_pressed){
            servo2.setPosition(0);
        }
        else{
            servo2.setPosition(0.5);
        }
    }
}
