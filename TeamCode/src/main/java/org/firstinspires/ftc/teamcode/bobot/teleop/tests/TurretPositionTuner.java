package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Config
@TeleOp(name = "Turret position tuner", group = "test")
public class TurretPositionTuner extends OpMode {
    public static double pos1 = 0.055;
    public static double pos2 = 0.055;

    private Servo servo1;
    private Servo servo2;
    private boolean usingPos2;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.FORWARD);
        usingPos2 = false;
        updatePosition();
    }

    @Override
    public void init_loop() {
        updatePosition();
    }

    @Override
    public void loop() {
        if (gamepad1.a) usingPos2 = false;
        if (gamepad1.y) usingPos2 = true;
        updatePosition();
    }

    private void updatePosition() {
        double position = usingPos2 ? pos2 : pos1;
        boolean valid = position >= 0.0 && position <= 1.0;
        if (valid) {
            servo1.setPosition(position);
            servo2.setPosition(position);
        }

        telemetry.addData("Controls", "Y: pos2 | A: pos1 (after START)");
        telemetry.addData("Selected", usingPos2 ? "pos2" : "pos1");
        telemetry.addData("pos1", "%.6f", pos1);
        telemetry.addData("pos2", "%.6f", pos2);
        telemetry.addData("Delta (pos2 - pos1)", "%.6f", pos2 - pos1);
        telemetry.addData("Status", valid ? "Command applied" : "Invalid position: use 0..1; holding last command");
        telemetry.addData("Servo 1 command", "%.6f", servo1.getPosition());
        telemetry.addData("Servo 2 command", "%.6f", servo2.getPosition());
        telemetry.addLine("Commands are not measured turret angles. Measure rotation physically.");
        telemetry.update();
    }
}
