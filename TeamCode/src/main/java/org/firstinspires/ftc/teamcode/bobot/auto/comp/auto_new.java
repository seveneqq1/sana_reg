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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.bobot.util.RegShooter;

@Autonomous(name = "Alikh auto new")
public class auto_new extends LinearOpMode {
    private RobotNew r;
    private Follower follower;
    private double shooterTPSOffset = 27;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.5, 7.8, 180);
    private final Pose shootpreload = poseFactory.of(55.5, 18, 180);
    private final Pose shootpreloadgarden = poseFactory.of(8.5, 8.5, 270);
    private final Pose shootpreloadgardenControl1 = poseFactory.of(36, 48, 0);
    private final Pose shootpreloadgardenControl2 = poseFactory.of(8, 32, 0);
    private final Pose shootpreloadgardenControl3 = poseFactory.of(16, 16, 0);
    private final Pose shootgardenStart = poseFactory.of(7.5, 8, 270);
    private final Pose shootgarden = poseFactory.of(47.5, 121.5, 90);
    private final Pose shootgardenControl1 = poseFactory.of(28, 18, 0);
    private final Pose shootgardenControl2 = poseFactory.of(16, 122, 0);
    private final Pose takeflower1Start = poseFactory.of(47.5, 121.5, 90);
    private final Pose takeflower1 = poseFactory.of(47.5, 128.6, 90);
    private final Pose moveflower1 = poseFactory.of(47.5, 120, 90);
    private final Pose back1shoot = poseFactory.of(47.5, 120.5, 90);
    private final Pose goflower2Start = poseFactory.of(47.5, 115, 90);
    private final Pose goflower2 = poseFactory.of(19.5, 46.5, 180);
    private final Pose goflower2Control1 = poseFactory.of(23, 106, 0);
    private final Pose goflower2Control2 = poseFactory.of(27.5, 43, 0);
    private final Pose flowertake2 = poseFactory.of(13.5, 47.5, 180);
    private final Pose flowermove2 = poseFactory.of(20.5, 47.5, 180);
    private final Pose shootflower2Start = poseFactory.of(14.5, 47.5, 180);
    private final Pose shootflower2Control1 = poseFactory.of(28, 42, 0);
    private final Pose shootflower2Control2 = poseFactory.of(18, 121, 0);
    private final Pose shootflower2 = poseFactory.of(54, 120, 70);
    private final Pose park = poseFactory.of(8.5, 122.5, 0); //8.5 92 park bottom
    private final Pose parkSegment1Heading = poseFactory.of(8.5, 122.5, 70);
    private final Pose parkSegment2Start = poseFactory.of(8.5, 122.5, 70);
    private final Pose parkSegment2End = poseFactory.of(8.5, 122.5, 0);
    private SequentialCommandGroup autoRoutine() {
        return new SequentialCommandGroup(
                nudgee(r, -0.3),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, shootpreload()),
                        spinUp(r)
                        ),
                new WaitCommand(100),
                shoot(r),
                new InstantCommand(() -> shooterTPSOffset = 30),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, shootpreloadgarden()),
                        intake(r),
                        spinUp(r)
                ),
                new WaitCommand(700),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, shootgarden()),
                        new SequentialCommandGroup(
                                new WaitCommand(3000),
                                new ParallelCommandGroup(
                                        intakeFlower(r),
                                        shoot(r)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        intake(r),
                        new SequentialCommandGroup(
                                new WaitCommand(60),
                                new FollowPathCommand(follower, takeflower1())
                        ),
                        spinUp(r)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(120),
                        new FollowPathCommand(follower, moveflower1_1()),
                        new WaitCommand(80),
                        new FollowPathCommand(follower, moveflower1_2())
                ),
                intake(r),
                new WaitCommand(400),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, back1shoot()),
                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                shoot(r)
                        )
                ),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, goflower2()),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                intake(r)
                        )
                ),
                new WaitCommand(520),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, flowertake2()),
                        intake(r)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(120),
                        new FollowPathCommand(follower, moveflower2_1()),
                        new WaitCommand(120),
                        new FollowPathCommand(follower, moveflower2_2())
                ),
                intake(r),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, shootflower2()),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                closeFlower(r)
                        ),
                        spinUp(r)
                ),
                new WaitCommand(200),
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
                    double defaultTPS = RegShooter.velForDist(
                            pose.distance(target), RobotNew.ShooterNew.baseVel);
                    r.shooter.setTargetTPS(defaultTPS > 0 ? defaultTPS + shooterTPSOffset : 0);
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
                    r.turnOff();
                } finally {
                    scheduler.registerSubsystem(r.drive);
                }
            }
        } finally {
            r.rgb.idle();
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

    public Path moveflower1_1(){
        return Paths.line(takeflower1, moveflower1).linear(takeflower1, moveflower1);
    }

    public Path moveflower1_2(){
        return Paths.line(moveflower1, takeflower1).linear(takeflower1, moveflower1);
    }

    public Path back1shoot() {
        return Paths.line(takeflower1, back1shoot).constant(back1shoot);
    }

    public Path goflower2() {
        return Paths.curve(goflower2Start, goflower2Control1, goflower2Control2,
                goflower2).linear(goflower2Start, goflower2);
    }

    public Path flowertake2() {
        return Paths.line(goflower2, flowertake2).constant(flowertake2);
    }

    public Path moveflower2_1(){
        return Paths.line(flowertake2, flowermove2).linear(flowertake2, flowermove2);
    }

    public Path moveflower2_2(){
        return Paths.line(flowermove2, flowertake2).linear(flowertake2, flowermove2);
    }

    public Path shootflower2() {
        return Paths.curve(shootflower2Start, shootflower2Control1, shootflower2Control2, shootflower2).linear(shootflower2Start, shootflower2);
    }

    public Path park() {
        return Paths.line(shootflower2, park).heading(Interpolator.piecewise().until(0.2971, Interpolator.constant(parkSegment1Heading)).until(1, Interpolator.linear(parkSegment2Start, parkSegment2End)));
    }
}
