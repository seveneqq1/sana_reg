package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "bekassss")
public class turretS extends OpMode {
    Servo servo1, servo2;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
    }

    @Override
    public void loop() {
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
