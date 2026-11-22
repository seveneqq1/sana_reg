package org.firstinspires.ftc.teamcode.bobot.auto.comp;

import static org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotCommands.*;

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

import org.firstinspires.ftc.teamcode.bobot.Robot;
import org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

@Autonomous(name = "test_paths", group = "Autonomous")
public class test_only_path extends LinearOpMode {
    Robot r;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorY(71);

    private final Pose start = poseFactory.of(56.1446, 7.8917, 180);
    private final Pose to_garden = poseFactory.of(9, 7.9, 180);
    private final Pose shoot1Start = poseFactory.of(9, 7.9, 180);
    private final Pose shoot1 = poseFactory.of(46.5, 120, 90);
    private final Pose shoot1Control1 = poseFactory.of(30, 110, 0);
    private final Pose flowerStart = poseFactory.of(46.5, 120, 90);
    private final Pose flower = poseFactory.of(46.5, 132, 90);
    private final Pose parkStart = poseFactory.of(46.5, 132, 90);
    private final Pose park = poseFactory.of(7.5, 120, 0);
    private final Pose parkControl1 = poseFactory.of(40, 116, 0);

    private SequentialCommandGroup autoRoutine() {
        return new SequentialCommandGroup(
                spinUp(r),
                shoot(r, 750),
                new ParallelCommandGroup(
                        intake(r),
                        new FollowPathCommand(follower, to_garden())
                ),
                new ParallelCommandGroup(
                        spinUp(r),
                        new FollowPathCommand(follower, shoot1())
                ),
                shoot(r, 750),
                new ParallelCommandGroup(
                        intakeFlower(r),
                        intake(r),
                        new FollowPathCommand(follower, flower()),
                        spinUp(r)
                ),
                shoot(r, 750),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, park()),
                        new SequentialCommandGroup(
                                new WaitCommand(240),
                                closeFlower(r)
                        )
                ),
                RobotCommands.idle(r)
        );
    }

    @Override
    public void runOpMode() {
        r = Robot.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
                Pose target = Turret.selectFieldTarget(pose);
                r.turret.face(target, pose, follower.velocity().omega);
                scheduler.run();

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
                scheduler.cancel(routine);
                follower.stop();
                follower.update();
                r.saveEnd();
            } finally {
                scheduler.registerSubsystem(r.drive);
            }
        }
    }

    public Path to_garden() {
        return Paths.line(start, to_garden).linear(start, to_garden);
    }

    public Path shoot1() {
        return Paths.curve(shoot1Start, shoot1Control1, shoot1).linear(shoot1Start, shoot1);
    }

    public Path flower() {
        return Paths.line(flowerStart, flower).linear(flowerStart, flower);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
