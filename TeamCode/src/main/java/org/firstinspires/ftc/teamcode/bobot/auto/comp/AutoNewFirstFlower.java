package org.firstinspires.ftc.teamcode.bobot.auto.comp;

import static org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

@Autonomous(name = "Alikh auto new first flower")
public class AutoNewFirstFlower extends LinearOpMode {
    private RobotNew r;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.5, 7.8, 180);
    private final Pose shootpreload = poseFactory.of(55.5, 22.5, 180);
    private final Pose shootpreloadgarden = poseFactory.of(7.5, 8, 270);
    private final Pose shootpreloadgardenControl1 = poseFactory.of(36, 48, 0);
    private final Pose shootpreloadgardenControl2 = poseFactory.of(8, 32, 0);
    private final Pose shootpreloadgardenControl3 = poseFactory.of(16, 16, 0);
    private final Pose shootgardenStart = poseFactory.of(7.5, 8, 270);
    private final Pose shootgarden = poseFactory.of(46.5, 121.5, 90);
    private final Pose shootgardenControl1 = poseFactory.of(28, 18, 0);
    private final Pose shootgardenControl2 = poseFactory.of(16, 122, 0);
    private final Pose takeflower1Start = poseFactory.of(46.5, 121.5, 90);
    private final Pose takeflower1 = poseFactory.of(46.5, 128, 90);
    private final Pose back1shoot = poseFactory.of(46.5, 112, 90);
    private final Pose park = poseFactory.of(8.5, 122.5, 0);

    private SequentialCommandGroup autoRoutine() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, shootpreload()),
                        spinUp(r)
                        ),
                new WaitCommand(200),
                shoot(r),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, shootpreloadgarden()),
                        intake(r),
                        spinUp(r)
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, shootgarden()),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                shoot(r)
                        )
                ),
                new ParallelCommandGroup(
                        intakeFlower(r),
                        intake(r),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new FollowPathCommand(follower, takeflower1())
                        ),
                        spinUp(r)
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, back1shoot()),
                        new SequentialCommandGroup(
                                new WaitCommand(400),
                                shoot(r)
                        )
                ),
                closeFlower(r),
                new FollowPathCommand(follower, park()),
                RobotNewCommands.idle(r)
        );
    }

    @Override
    public void runOpMode() {
        r = RobotNew.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            r.init(hardwareMap, start);
            follower = r.drive.getFollower();
            follower.update();

            waitForStart();
            if (isStopRequested()) return;

            CommandScheduler scheduler = CommandScheduler.getInstance();
            SequentialCommandGroup routine = autoRoutine();
            scheduler.unregisterSubsystem(r.drive);
            try {
                r.turret.on();
                scheduler.schedule(routine);
                while (opModeIsActive()) {
                    follower.update();
                    Pose pose = follower.pose();
                    Pose target = Turret.selectWestFieldTarget(pose);
                    r.turret.face(target, pose, follower.velocity().omega);
                    scheduler.run();

                    r.limelight.debug(telemetry);
                    telemetry.addData("Vision trim (deg)", r.turret.getVisionTrimDegrees());
                    telemetry.addData("x", pose.x());
                    telemetry.addData("y", pose.y());
                    telemetry.addData("heading", pose.heading());
                    telemetry.addData("Goal", target);
                    telemetry.addData("Turret target yaw (deg)", Math.toDegrees(r.turret.getYaw()));

                    telemetry.update();
                }
            } finally {
                r.turret.off();
                try {
                    scheduler.cancelAll();
                    follower.stop();
                    follower.update();
                    r.saveEnd();
                    r.turnOff();
                } finally {
                    scheduler.registerSubsystem(r.drive);
                }
            }
        } finally {
            r.limelight.stop();
        }
    }

    public Path shootpreload() {
        return Paths.line(start, shootpreload).linear(start, shootpreload);
    }

    public Path shootpreloadgarden() {
        return Paths.curve(shootpreload, shootpreloadgardenControl1, shootpreloadgardenControl2,
                shootpreloadgardenControl3, shootpreloadgarden).linear(shootpreload, shootpreloadgarden);
    }

    public Path shootgarden() {
        return Paths.curve(shootgardenStart, shootgardenControl1, shootgardenControl2,
                shootgarden).linear(shootgardenStart, shootgarden);
    }

    public Path takeflower1() {
        return Paths.line(takeflower1Start, takeflower1).linear(takeflower1Start, takeflower1);
    }

    public Path back1shoot() {
        return Paths.line(takeflower1, back1shoot).constant(back1shoot);
    }

    public Path park() {
        return Paths.line(back1shoot, park).linear(back1shoot, park);
    }
}
