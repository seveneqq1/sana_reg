package org.firstinspires.ftc.teamcode.bobot.auto.comp;

import static org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.firstinspires.ftc.teamcode.bobot.auto.commands.RobotNewCommands;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

@Disabled
public class autoDumb extends LinearOpMode {
    RobotNew r;
    Follower f;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(13.8937, 133.9331, 180);
    private final Pose path1 = poseFactory.of(8.7904, 120.7872, 180);

    private SequentialCommandGroup autoRoutine(){
        return new SequentialCommandGroup(
                spinUp(r),
                setBaseVel(r, 1531),
                shoot(r),
                new FollowPathCommand(f, path1()),
                RobotNewCommands.idle(r)
        );
    }

    @Override
    public void runOpMode() {
        r = RobotNew.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            r.init(hardwareMap, start);
            f = r.drive.getFollower();
            f.update();

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
                    f.update();
                    Pose pose = f.pose();
                    Pose target = Turret.selectWestFieldTarget(pose);
                    r.turret.face(target, pose, f.velocity().omega);
                    scheduler.run();

                    long now = System.nanoTime();
                    if (now - lastTelemetryNanos >= 100_000_000L) {
                        lastTelemetryNanos = now;
                        r.limelight.debug(telemetry);
                        telemetry.addData("Vision trim (deg)", r.turret.getVisionTrimDegrees());
                        telemetry.addData("x", pose.x());
                        telemetry.addData("y", pose.y());
                        telemetry.addData("heading", pose.heading());
                        telemetry.addData("Goal", target);
                        telemetry.addData("Turret target yaw (deg)", Math.toDegrees(r.turret.getYaw()));

                        telemetry.update();
                    }
                }
            } finally {
                r.turret.off();
                try {
                    scheduler.cancel(routine);
                    f.stop();
                    f.update();
                    r.saveEnd();
                } finally {
                    scheduler.registerSubsystem(r.drive);
                }
            }
        } finally {
            r.limelight.stop();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }
}
