package org.firstinspires.ftc.teamcode.bobot.teleop.comp;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.bobot.teleop.tests.DebugNew;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class redTeleop extends DebugNew {
    @Override
    protected Pose selectFieldTarget(Pose pose) {
        return Turret.selectFieldTarget(pose, false);
    }
}
