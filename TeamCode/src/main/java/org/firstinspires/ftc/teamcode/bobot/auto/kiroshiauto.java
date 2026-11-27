package org.firstinspires.ftc.teamcode.bobot.auto;

import static org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotCommands.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.bobot.Robot;
import org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

@Disabled
@Autonomous(name = "kiroshi", group = "Autonomous")
public class kiroshiauto extends LinearOpMode {
    Robot r;

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 180);
    private final Pose goflower = poseFactory.of(17, 46.5, 180);
    private final Pose takeflowerStart = poseFactory.of(17, 46.5, 180);
    private final Pose takeflower = poseFactory.of(10, 46.5, 180);
    private final Pose point3Start = poseFactory.of(10, 46.5, 180);
    private final Pose point3 = poseFactory.of(40, 120, -108);
    private final Pose point3Control1 = poseFactory.of(28, 45, 0);
    private final Pose point3Control2 = poseFactory.of(25, 120, 0);
    private final Pose point4Start = poseFactory.of(40, 120, -108);
    private final Pose point4 = poseFactory.of(8.5, 15, -90);
    private final Pose point5Start = poseFactory.of(8.5, 15, 270);
    private final Pose point5 = poseFactory.of(8.5, 9.0252, -90);
    private final Pose point6Start = poseFactory.of(8.5, 9.0252, -90);
    private final Pose point6 = poseFactory.of(60, 15, -90);
    private final Pose point7 = poseFactory.of(9.7178, 93.3303, 104.6812);
    private final Pose point7Control1 = poseFactory.of(25, 35, 0);

    private SequentialCommandGroup autoRoutine() {
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, goflower()),
                new FollowPathCommand(follower, takeflower()),
                new FollowPathCommand(follower, path3()),
                new FollowPathCommand(follower, path4()),
                new FollowPathCommand(follower, path5()),
                new FollowPathCommand(follower, path6()),
                new FollowPathCommand(follower, path7()),
                RobotCommands.idle(r)
        );
    }

    @Override
    public void runOpMode() {
        r = Robot.getInstance();
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
                long lastTelemetryNanos = System.nanoTime() - 100_000_000L;
                while (opModeIsActive()) {
                    follower.update();
                    Pose pose = follower.pose();
                    Pose target = Turret.selectFieldTarget(pose);
                    r.turret.face(target, pose, follower.velocity().omega);
                    scheduler.run();

                    long now = System.nanoTime();
                    if (now - lastTelemetryNanos >= 100_000_000L) {
                        lastTelemetryNanos = now;
                        telemetry.addData("Pose", pose);

                        if (follower.currentPath() != null) {
                            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                            telemetry.addData("Path number", follower.pathIndex());
                        }

                        telemetry.update();
                    }
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
        } finally {
            r.limelight.stop();
        }
    }

    public Path goflower() {
        return Paths.line(start, goflower).linear(start, goflower);
    }

    public Path takeflower() {
        return Paths.line(takeflowerStart, takeflower).linear(takeflowerStart, takeflower);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3Control2, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.curve(point6, point7Control1, point7).tangent();
    }
}