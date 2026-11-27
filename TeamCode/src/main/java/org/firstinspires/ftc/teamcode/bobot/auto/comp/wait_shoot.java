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
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

@Autonomous(name = "wait  shoot  park")
public class wait_shoot extends LinearOpMode {
    private RobotNew r;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(62.8, 133.5, 180);
    private final Pose path1 = poseFactory.of(8.6992, 120.37, 180); //8.5 92 bottom park
    private final Pose path1Control1 = poseFactory.of(65.4193, 112.3903, 0);

    private SequentialCommandGroup autoRoutine() {
        return new SequentialCommandGroup(
                useDistanceTPS(r),
                spinUp(r),
                new WaitCommand(8000),
                shoot(r),
                new WaitCommand(15000),
                new FollowPathCommand(follower, path1()),
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

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }
}
