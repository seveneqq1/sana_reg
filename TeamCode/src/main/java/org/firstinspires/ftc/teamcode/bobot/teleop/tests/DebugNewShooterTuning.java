package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "DebugNew Shooter Distance Tuning", group = "Tests")
public class DebugNewShooterTuning extends DebugNew {
    public static double tuningTPS = 1234;

    @Override
    protected double getFixedShooterTargetTPS() {
        return tuningTPS;
    }
}
