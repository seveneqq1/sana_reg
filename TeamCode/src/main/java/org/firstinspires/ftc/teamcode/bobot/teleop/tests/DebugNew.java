package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Rgb;
import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.firstinspires.ftc.teamcode.bobot.teleop.BekaBaseNew;
import org.firstinspires.ftc.teamcode.bobot.util.RegShooter;

@Config
@TeleOp(name = "Alimzhan debug")
public class DebugNew extends BekaBaseNew {

    public static double targetX = 72, targetY = 62;
    private long lastTelemetryNanos;
    public static double slowMultiplier = 0.6;
    public static double manualTurretDegreesPerSecond = 120;
    public static double shooterTolerance = 120;
    public static double stopperOpenDelay = 0.15;
    public static double feedKickTime = 0;
    public static double shootTime = 2.4;

    private enum Mode { IDLE, INTAKE, OUTTAKE, SHOOT }

    private final Rgb rgb = new Rgb();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private Mode mode = Mode.IDLE;
    private Pose target;
    private Pose correctedTarget;
    private boolean tracking = true;
    private boolean manualLastLoop;
    private boolean feeding;
    private boolean compensationTriggered = false;
    private double previousHeading;
    private boolean transferOverloaded;
    private long lastRumbleNanos;
    private boolean distanceBasedShooting;

    @Override
    protected void onStart() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rgb.init(hardwareMap);
        transferOverloaded = false;
        lastRumbleNanos = 0;
        r.turnOn();
        r.shooter.turnOn();
        r.shooter.setStandby(false);
        r.shooter.setFeeding(false);
        initializeTracking();
        distanceBasedShooting = false;
        updateShooterTarget(r.drive.getPose().distance(target));
        lastTelemetryNanos = System.nanoTime() - 100_000_000L;
    }

    @Override
    protected void onUpdate() {
        if (pressed(GamepadKeys.Button.DPAD_UP)) resetTracking();
        if (second_driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            distanceBasedShooting = !distanceBasedShooting;
        }

        Pose pose = r.drive.getPose();

        if (target.x() != targetX || target.y() != targetY) {
            target = new Pose(targetX, targetY, 0);
            correctedTarget = null;
        }

        double headingRate = deltaTime > 0
                ? Turret.normalizeAngle(pose.heading() - previousHeading) / deltaTime : 0;

        previousHeading = pose.heading();
        updateTurret(pose, headingRate);

        double distanceToGoal = pose.distance(target);
        updateShooterTarget(distanceToGoal);
        double velocity = Math.abs(r.shooter.getVel());

        updateIntake(velocity);
        rgb.periodic();
        updateDrive();

        boolean overloaded = r.intake.getTransferCurrentMilliamps() > 4000.0;
        long rumbleNow = System.nanoTime();
        if (overloaded) {
            if (!transferOverloaded || rumbleNow - lastRumbleNanos >= 100_000_000L) {
                gamepad1.rumble(1.0, 1.0, 150);
                lastRumbleNanos = rumbleNow;
            }
        } else if (transferOverloaded) {
            gamepad1.stopRumble();
        }
        transferOverloaded = overloaded;

        long now = System.nanoTime();
        if (now - lastTelemetryNanos >= 100_000_000L) {
            lastTelemetryNanos = now;
            double voltage = r.shooter.getVoltage();

            telemetry.addData("Pose", pose);
            telemetry.addData("Distance to goal (in)", "%.2f", distanceToGoal);
            telemetry.addData("Shooting mode", distanceBasedShooting ? "DISTANCE" : "FIXED");
            if (distanceBasedShooting) {
                telemetry.addData("Distance equation", !RegShooter.calibrated
                        ? "Not calibrated - using fixed TPS"
                        : RegShooter.useQuadratic ? "QUADRATIC" : "LINEAR");
            }
            telemetry.addData("Shooter TPS", "%.0f / %.0f", velocity, r.shooter.getActiveTargetTPS());
            telemetry.addData("Shooter Ready", isShooterReady(velocity, shooterTolerance));
            telemetry.addData("Voltage", voltage);
            telemetry.addData("Loop period (ms)", "%.2f", deltaTime * 1000);
            telemetry.addData("Turret out of range", r.turret.isOutOfRange());
            telemetry.update();
        }
    }

    protected double getFixedShooterTargetTPS() {
        return RobotNew.ShooterNew.baseVel;
    }

    private void updateShooterTarget(double distanceInches) {
        double fixedTPS = getFixedShooterTargetTPS();
        r.shooter.setTargetTPS(distanceBasedShooting
                ? RegShooter.velForDist(distanceInches, fixedTPS) : fixedTPS);
    }

    private void resetTracking() {
        r.drive.resetPos();
        initializeTracking();
    }

    private void initializeTracking() {
        target = new Pose(targetX, targetY, 0);
        correctedTarget = null;
        r.turret.resetAimTrim();
        previousHeading = r.drive.getPose().heading();
        tracking = true;
        manualLastLoop = false;
    }

    private void updateDrive() {
        if (first_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
            r.drive.drive(0, 0, 0);
            return;
        }

        double scale = first_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2
                ? slowMultiplier : 1.0;

        r.drive.drive(first_driver.getLeftY() * scale, first_driver.getLeftX() * scale,
                first_driver.getRightX() * scale);
    }

    private void updateTurret(Pose pose, double headingRate) {
        if (pressed(GamepadKeys.Button.DPAD_LEFT)) {
            tracking = !tracking;
            if (!tracking) {
                correctedTarget = null;
                r.turret.setYaw(0);
            }
        }

        boolean left = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean right = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean manual = left != right;

        if (manual) {
            double step = Math.toRadians(manualTurretDegreesPerSecond * Math.min(deltaTime, 0.1));
            r.turret.addYaw(left ? step : -step);
        } else if (tracking) {
            if (manualLastLoop) lockTargetHere(pose);
            r.turret.face(correctedTarget != null ? correctedTarget : target, pose, headingRate, target);
        }

        manualLastLoop = manual && tracking;
    }

    private void lockTargetHere(Pose pose) {
        double angle = pose.heading() + r.turret.getYaw();
        double sin = Math.sin(angle);
        double distance = Math.abs(sin) > 0.2 ? (target.y() - pose.y()) / sin : -1;

        if (distance > 0) {
            correctedTarget = new Pose(pose.x() + distance * Math.cos(angle), target.y(), 0);
        } else {
            distance = Math.max(1, Math.hypot(target.x() - pose.x(), target.y() - pose.y()));
            correctedTarget = new Pose(pose.x() + distance * Math.cos(angle),
                    pose.y() + distance * sin, 0);
        }
        r.turret.resetAimTrim();
    }

    private void updateIntake(double velocity) {
        if (pressed(GamepadKeys.Button.X)) setMode(Mode.INTAKE);
        if (pressed(GamepadKeys.Button.B)) setMode(Mode.OUTTAKE);
        if (pressed(GamepadKeys.Button.A)) setMode(Mode.IDLE);
        if (pressed(GamepadKeys.Button.Y)) setMode(Mode.SHOOT);

        if (mode != Mode.SHOOT) return;

        if (!feeding) {
            if (!isShooterReady(velocity, shooterTolerance)) {
                return;
            }

            r.shooter.setFeeding(true);
            feedTimer.reset();
            feeding = true;
        }

        double elapsed = feedTimer.seconds();

        if (elapsed >= shootTime) {
            setMode(Mode.INTAKE);

        } else if (elapsed < stopperOpenDelay) {
            r.intake.spinOff();

        } else {
            if (!compensationTriggered) {
                r.shooter.anticipateShot();
                compensationTriggered = true;
            }

            if (elapsed < stopperOpenDelay + feedKickTime) {
                r.intake.spinShoot();
            } else {
                r.intake.spinSlowShoot();
            }
        }
    }

    private void setMode(Mode next) {
        switch (next) {
            case INTAKE: rgb.intake(); break;
            case SHOOT: rgb.shoot(); break;
            default: rgb.idle();
        }
        mode = next;
        feeding = false;
        compensationTriggered = false;
        r.shooter.setFeeding(false);

        switch (mode) {
            case INTAKE: r.intake.spinIn(); break;
            case OUTTAKE: r.intake.spinOut(); break;
            default: r.intake.spinOff();
        }
    }

    private boolean pressed(GamepadKeys.Button button) {
        return first_driver.wasJustPressed(button);
    }
}
