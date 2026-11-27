package org.firstinspires.ftc.teamcode.bobot.auto.comp;

import static org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

@Autonomous(name = "Beka 2 stable")
public class auto_2_stable extends LinearOpMode {
    private RobotNew r;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.5, 8, 180);
    private final Pose prelaod = poseFactory.of(55.5, 18, 180);
    private final Pose gogarden1 = poseFactory.of(17.4981, 47.5, 180);
    private final Pose takegarden1 = poseFactory.of(14, 47.5, 180);
    private final Pose movegarden1_1 = poseFactory.of(24, 47.5, 180);
    private final Pose movegarden1_2 = poseFactory.of(14, 47.5, 180);
    private final Pose goflower2Start = poseFactory.of(14, 47.5, 180);
    private final Pose goflower2 = poseFactory.of(46.5, 122.5, 90);
    private final Pose goflower2Control1 = poseFactory.of(32.5, 42, 0);
    private final Pose goflower2Control2 = poseFactory.of(21.5, 120, 0);
    private final Pose takeflower2 = poseFactory.of(46.5, 128, 90);
    private final Pose moveflower2 = poseFactory.of(46.5, 120, 90);
    private final Pose takegarden = poseFactory.of(7.5, 8.5, 270);
    private final Pose takegardenControl1 = poseFactory.of(32, 120, 0);
    private final Pose takegardenControl2 = poseFactory.of(22, 22, 0);
    private final Pose takegardenControl3 = poseFactory.of(7.5, 24, 0);
    private final Pose takegardenSegment1Start = poseFactory.of(7.5, 8.5, 90);
    private final Pose takegardenSegment1End = poseFactory.of(7.5, 8.5, 252);
    private final Pose takegardenSegment2Heading = poseFactory.of(7.5, 8.5, 252);
    private final Pose takegardenSegment3Start = poseFactory.of(7.5, 8.5, 252);
    private final Pose takegardenSegment3End = poseFactory.of(7.5, 8.5, 270);
    private final Pose shootgardenStart = poseFactory.of(7.5, 8.5, 270);
    private final Pose shootgarden = poseFactory.of(49.0949, 118.2255, 180);
    private final Pose shootgardenControl1 = poseFactory.of(32.5, 112.5, 0);
    private final Pose shootgardenControl2 = poseFactory.of(37.1077, 104.7901, 0);
    private final Pose parkStart = poseFactory.of(49.0949, 118.2255, 180);
    private final Pose park = poseFactory.of(8.5, 122.5, 180);

    private SequentialCommandGroup autoRoutine() {
        return new SequentialCommandGroup(
                nudgee(r, -0.3),
                new WaitCommand(800),
                new ParallelCommandGroup(
                        spinUp(r),
                        new FollowPathCommand(follower, shootpreload())
                ),
                new WaitCommand(200),
                shoot(r),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, gogarden1()),
                        intakeFlower(r),
                        intake(r)
                ),
                new WaitCommand(160),
                new ParallelCommandGroup(
                        intake(r),
                        new FollowPathCommand(follower, takegarden1()),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new FollowPathCommand(follower, movegarden1_1()),
                                new WaitCommand(100),
                                new FollowPathCommand(follower, movegarden1_2())
                        )
                ),
                new WaitCommand(800),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(900),
                                intake(r),
                                spinUp(r)
                        ),
                        new FollowPathCommand(follower, goflower2()),
                        setFlower(r),
                        nudgee(r, 0.1)
                ),
                new WaitCommand(400),
                intake(r),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, takeflower2()),
                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                new FollowPathCommand(follower, moveflower2_1()),
                                new WaitCommand(200),
                                new FollowPathCommand(follower, moveflower2_2())
                        ),
                        shoot(r)
                ),
                shoot(r),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, takegarden()),
                        intake(r),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                spinUp(r)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                closeFlower(r)
                        )
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, shootgarden()),
                        spinUp(r)
                ),
                shoot(r),
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
            r.rgb.init(hardwareMap);
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
                    Pose target = Turret.selectWestFieldTarget(pose);
                    r.turret.face(target, pose, follower.velocity().omega);
                    scheduler.run();
                    r.rgb.periodic();

                    long now = System.nanoTime();
                    if (now - lastTelemetryNanos >= 100_000_000L) {
                        lastTelemetryNanos = now;
                        telemetry.addData("Pose", pose);
                        telemetry.addData("Distance to goal (in)", "%.2f", pose.distance(target));
                        telemetry.addData("Shooter TPS", "%.0f / %.0f",
                                r.shooter.getVel(), r.shooter.getActiveTargetTPS());
                        telemetry.update();
                    }
                }
            } finally {
                r.turret.off();
                try {
                    scheduler.cancelAll();
                    follower.stop();
                    follower.update();
                    r.saveEnd();
                } finally {
                    try {
                        r.turnOff();
                    } finally {
                        scheduler.registerSubsystem(r.drive);
                    }
                }
            }
        } finally {
            r.rgb.idle();
            r.limelight.stop();
        }
    }

    public Path shootpreload() { return Paths.line(start, prelaod).linear(start, prelaod); }
    public Path gogarden1() {
        return Paths.line(prelaod, gogarden1).linear(prelaod, gogarden1);
    }

    public Path takegarden1() {
        return Paths.line(gogarden1, takegarden1).constant(takegarden1);
    }
    public Path movegarden1_1() { return Paths.line(takegarden1, movegarden1_1).linear(takegarden1, movegarden1_1); }
    public Path movegarden1_2() { return Paths.line(movegarden1_1, takegarden1).linear(takegarden1, movegarden1_1); }

    public Path goflower2() {
        return Paths.curve(goflower2Start, goflower2Control1, goflower2Control2,
                goflower2).linear(goflower2Start, goflower2);
    }

    public Path takeflower2() {
        return Paths.line(goflower2, takeflower2).constant(takeflower2);
    }

    public Path moveflower2_1() { return Paths.line(takeflower2, moveflower2).linear(takeflower2, moveflower2); }
    public Path moveflower2_2() { return Paths.line(moveflower2, takeflower2).linear(takeflower2, moveflower2); }

    public Path takegarden() {
        return Paths.curve(takeflower2, takegardenControl1, takegardenControl2,
                takegardenControl3, takegarden).heading(Interpolator.piecewise()
                        .until(0.3, Interpolator.linear(takegardenSegment1Start, takegardenSegment1End))
                        .until(0.65, Interpolator.constant(takegardenSegment2Heading))
                        .until(1, Interpolator.linear(takegardenSegment3Start, takegardenSegment3End)));
    }

    public Path shootgarden() {
        return Paths.curve(shootgardenStart, shootgardenControl1, shootgardenControl2,
                shootgarden).linear(shootgardenStart, shootgarden);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
