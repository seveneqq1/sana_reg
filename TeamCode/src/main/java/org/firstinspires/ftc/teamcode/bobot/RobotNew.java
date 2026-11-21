package org.firstinspires.ftc.teamcode.bobot;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Drive;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Flower;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.bobot.subsystem.ShooterNew;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

public class RobotNew {
    private static final RobotNew INSTANCE = new RobotNew();
    private static Pose defaultPose = new Pose(135, 9, Math.PI / 2);

    public final Drive drive = new Drive();
    public final Intake intake = new Intake();
    public final ShooterNew shooter = new ShooterNew();
    public final Turret turret = new Turret();
    public final Flower flower = new Flower();

    private RobotNew() {}

    public static RobotNew getInstance() {
        return INSTANCE;
    }

    public void init(HardwareMap hw) {
        init(hw, defaultPose);
    }

    public void init(HardwareMap hw, Pose startPose) {
        drive.init(hw, startPose);
        intake.init(hw);
        shooter.init(hw);
        shooter.turnOff();
        turret.init(hw);
        turret.off();
        flower.init(hw);
        flower.closeFlower();
    }
    public void turnOn() {
        shooter.turnOn();
        turret.on();
    }

    public void turnOff() {
        intake.spinOff();
        shooter.turnOff();
        turret.off();
        drive.drive(0.0, 0.0, 0.0);
        drive.periodic();
        flower.closeFlower();
    }

    public void periodic() {
        drive.periodic();
        intake.periodic();
        shooter.periodic();
        turret.periodic();
    }

    public void saveEnd() {
        defaultPose = drive.getPose();
    }
}
