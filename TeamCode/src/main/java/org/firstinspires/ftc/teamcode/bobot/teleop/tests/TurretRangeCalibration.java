package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

@TeleOp(name = "Turret range calibration", group = "test")
public class TurretRangeCalibration extends OpMode {
    private ServoEx servo1, servo2;
    private double position = 0.2;
    private boolean previousA, previousB;

    @Override
    public void init() {
        servo1 = new ServoEx(hardwareMap, "servo1");
        servo2 = new ServoEx(hardwareMap, "servo2");
        servo1.setInverted(Turret.reverseServo1);
        servo2.setInverted(Turret.reverseServo2);
        telemetry.addLine("START moves both turret servos to 0.2.");
        telemetry.addLine("A: move to 0.8. B: return to 0.2.");
    }

    @Override
    public void start() {
        previousA = gamepad1.a;
        previousB = gamepad1.b;
        setPosition(0.2);
    }

    @Override
    public void loop() {
        if (gamepad1.b && !previousB) {
            setPosition(0.2);
        } else if (gamepad1.a && !previousA) {
            setPosition(0.8);
        }
        previousA = gamepad1.a;
        previousB = gamepad1.b;

        telemetry.addData("Commanded servo position", position);
        telemetry.addLine("A: 0.8 | B: 0.2");
        telemetry.addLine("Measure total turret rotation between these positions.");
        telemetry.addLine("kV = 0.6 / measured degrees");
    }

    private void setPosition(double position) {
        this.position = position;
        servo1.set(position);
        servo2.set(position);
    }
}
