package org.firstinspires.ftc.teamcode.bobot.teleop.comp;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.bobot.teleop.tests.DebugNew;
import org.firstinspires.ftc.teamcode.bobot.util.AprilTagCluster;

@Disabled
@TeleOp(name = "Blue TeleOp", group = "Competition")
public class blueTeleop extends DebugNew {
    @Override
    protected void onInit() {
        super.onInit();
        r.turret.setVisionCluster(AprilTagCluster.BLUE_AUDIENCE);
    }

    @Override
    protected Pose selectFieldTarget(Pose pose) {
        return Turret.selectFieldTarget(pose, false);
    }
}
