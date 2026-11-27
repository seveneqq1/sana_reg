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
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.nio.file.Watchable;

@Autonomous(name = "test_paths_new 1 version", group = "Autonomous")
public class test_only_path_new extends LinearOpMode {
    RobotNew r;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.1446, 7.8917, 180);
    private final Pose shootpreload = poseFactory.of(56.1446, 24, 180);
    private final Pose togarden = poseFactory.of(7.7392, 8.46, -90.9051);
    private final Pose togardenControl1 = poseFactory.of(8.4538, 53.6931, 0);
    private final Pose togardenSegment1Start = poseFactory.of(7.7392, 8.46, 0);
    private final Pose togardenSegment1End = poseFactory.of(7.7392, 8.46, 0);
    private final Pose shoot1Start = poseFactory.of(9, 7.9, 180);
    private final Pose shoot1 = poseFactory.of(46.5, 118, 90);
    private final Pose shoot1Control1 = poseFactory.of(30, 110, 0);
    private final Pose flowerStart = poseFactory.of(46.5, 118, 90);
    private final Pose flower = poseFactory.of(46.5, 128, 90);
    private final Pose flowermovearound2 = poseFactory.of(46.5, 126.2, 90);
    private final Pose flowermovearound1 = poseFactory.of(46.5, 128, 90);
    private final Pose flower22 = poseFactory.of(24, 46.5, 180);
    private final Pose flower22Control1 = poseFactory.of(21, 102, 0);
    private final Pose point2 = poseFactory.of(14.5, 46.5, 180);
    private final Pose fromflower22parkStart = poseFactory.of(14.5, 46.5, 180);
    private final Pose fromflower22park = poseFactory.of(8.75, 122.58, 180);
    private final Pose fromflower22parkControl1 = poseFactory.of(26.7755, 44.1981, 0);
    private final Pose fromflower22parkControl2 = poseFactory.of(24.5625, 70.9378, 0);
    private final Pose fromflower22parkControl3 = poseFactory.of(24.6243, 84.5133, 0);
    private final Pose fromflower22parkControl4 = poseFactory.of(7.976, 76.9142, 0);
    private final Pose parkStart = poseFactory.of(46.5, 132, 90);
    private final Pose park = poseFactory.of(7.5, 120, 0);
    private final Pose parkControl1 = poseFactory.of(40, 116, 0);

    private SequentialCommandGroup autoRoutine() {
        return new SequentialCommandGroup(
                useDistanceTPS(r),
                spinUp(r),
                new FollowPathCommand(follower, shootPreload()),
                shoot(r),
                new ParallelCommandGroup(
                        intake(r),
                        new FollowPathCommand(follower, togarden())
                ),
                new ParallelCommandGroup(
                        spinUp(r),
                        new FollowPathCommand(follower, shoot1()),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                nudgee(r, -0.3)
                        )
                ),
                shoot(r),
                new ParallelCommandGroup(
                        intakeFlower(r),
                        intake(r),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new FollowPathCommand(follower, flower()),
                                        spinUp(r)
                                ),
                                new WaitCommand(50),
                                new FollowPathCommand(follower, bekamove()),
                                new WaitCommand(50),
                                new FollowPathCommand(follower, bekamove1())
                        )
                ),
                nudgee(r, 0.08),
                spinUp(r),
                shoot(r),
                closeFlower(r),
                new FollowPathCommand(follower, flower22()),
                intakeFlower(r),
                new WaitCommand(320),
                intake(r),
                new FollowPathCommand(follower, path2()),
                intake(r),
                new ParallelCommandGroup(
                        spinUp(r),
                        new FollowPathCommand(follower, fromflower22park()),
                        new SequentialCommandGroup(
                                new WaitCommand(400),
                                closeFlower(r)
                        )
                ),
                setBaseVel(r, 1850),
                nudgee(r, 0.3),
                nudgee(r, 0.3),
                new WaitCommand(200),
                shoot(r),
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
                    scheduler.cancel(routine);
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

    public Path shootPreload(){
        return Paths.line(start, shootpreload).linear(start, shootpreload);
    }

    public Path bekamove(){
        return Paths.line(flower, flowermovearound2).linear(flower, flowermovearound2);
    }
    public Path bekamove1(){
        return Paths.line(flowermovearound2, flowermovearound1).linear(flowermovearound2, flowermovearound1);
    }
    public Path togarden() {
        return Paths.curve(shootpreload, togardenControl1, togarden).heading(Interpolator.piecewise().until(0.3899, Interpolator.linear(togardenSegment1Start, togardenSegment1End)).until(1, Interpolator.tangent)) ;
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
    public Path flower22() {
        return Paths.curve(flowermovearound2, flower22Control1, flower22).linear(flowermovearound2, flower22);
    }

    public Path path2() {
        return Paths.line(flower22, point2).constant(point2);
    }

    public Path fromflower22park() {
        return Paths.curve(fromflower22parkStart, fromflower22parkControl1, fromflower22parkControl2, fromflower22parkControl3, fromflower22parkControl4, fromflower22park).linear(fromflower22parkStart, fromflower22park);
    }
}
