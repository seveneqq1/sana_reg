package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Rgb {
    private Servo servo;
    private long shootPressedAt = -1;

    public void init(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "rgb");
        idle();
    }

    public void intake() {
        shootPressedAt = -1;
        servo.setPosition(0.611);
    }

    public void shoot() {
        shootPressedAt = System.nanoTime();
        servo.setPosition(0.8);
    }

    public void idle() {
        shootPressedAt = -1;
        servo.setPosition(0);
    }

    public void afterShoot() {
        shootPressedAt = -1;
        servo.setPosition(0.5);
    }

    public void periodic() {
        if (shootPressedAt >= 0 && System.nanoTime() - shootPressedAt >= 1000_000_000L) {
            if(System.nanoTime() - shootPressedAt <= 2000_000_000L){
                afterShoot();
            } else {
                idle();
            }
        }
    }
}
