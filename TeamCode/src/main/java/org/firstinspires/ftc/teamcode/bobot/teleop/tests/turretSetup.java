package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret servos 0.5", group = "test")
public class turretSetup extends OpMode {

    Servo servo1, servo2;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void loop() {
        servo1.setPosition(0.5);
        servo2.setPosition(0.5);
    }
}
