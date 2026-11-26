package org.firstinspires.ftc.teamcode.bobot.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Rgb;
import org.firstinspires.ftc.teamcode.bobot.teleop.BekaBaseNew;

@Config
@TeleOp(name = "Alimzhan debug")
public class DebugNew extends BekaBaseNew {

    public static double targetX = 50, targetY = 50;
    VoltageSensor voltageSensor;
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
    private double min_Volts = 100;
    private boolean transferOverloaded;
    private long lastRumbleNanos;

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
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        lastTelemetryNanos = System.nanoTime() - 100_000_000L;
    }

    @Override
    protected void onUpdate() {
        if (pressed(GamepadKeys.Button.DPAD_UP)) resetTracking();

        Pose pose = r.drive.getPose();

        Pose fieldTarget = selectFieldTarget(pose);
        targetX = fieldTarget.x();
        targetY = fieldTarget.y();

        if (target.x() != targetX || target.y() != targetY) {
            target = new Pose(targetX, targetY, 0);
            correctedTarget = null;
        }

        double headingRate = deltaTime > 0
                ? Turret.normalizeAngle(pose.heading() - previousHeading) / deltaTime : 0;

        previousHeading = pose.heading();
        updateTurret(pose, headingRate);

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
            double voltage = voltageSensor.getVoltage();
            min_Volts = Math.min(min_Volts, voltage);

            r.limelight.debug(telemetry);
            telemetry.addData("Vision trim (deg)", r.turret.getVisionTrimDegrees());
            telemetry.addData("Pose", pose);
            telemetry.addData("Goal", correctedTarget != null ? correctedTarget : target);
            telemetry.addData("Shooter TPS", "%.0f / %.0f", velocity, r.shooter.getActiveTargetTPS());
            telemetry.addData("Shooter Ready", isShooterReady(velocity, shooterTolerance));
            telemetry.addData("Voltage", voltage);
            telemetry.addData("Minimum Voltage", min_Volts);
            telemetry.addData("dt", deltaTime);
            telemetry.addData("freq", deltaTime > 0 ? 1 / deltaTime : 0);
            telemetry.addData("shooter motor power", r.shooter.l.getPower());
            telemetry.addData("dist to hive", pose.distance(fieldTarget));
            telemetry.update();
        }
    }

    protected Pose selectFieldTarget(Pose pose) {
        return Turret.selectFieldTarget(pose);
    }

    private void resetTracking() {
        r.drive.resetPos();
        initializeTracking();
    }

    private void initializeTracking() {
        target = new Pose(targetX, targetY, 0);
        correctedTarget = null;
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
            if (manualLastLoop) lockTargetHere(pose, headingRate);
            r.turret.face(correctedTarget != null ? correctedTarget : target, pose, headingRate, target);
        }

        manualLastLoop = manual && tracking;
    }

    private void lockTargetHere(Pose pose, double headingRate) {
        double angle = pose.heading() + headingRate * Turret.turnLeadSeconds + r.turret.getYaw();
        double sin = Math.sin(angle);
        double distance = Math.abs(sin) > 0.2 ? (target.y() - pose.y()) / sin : -1;

        if (distance >= 1) {
            correctedTarget = new Pose(pose.x() + distance * Math.cos(angle), target.y(), 0);
        } else {
            distance = Math.max(1, Math.hypot(target.x() - pose.x(), target.y() - pose.y()));
            correctedTarget = new Pose(pose.x() + distance * Math.cos(angle),
                    pose.y() + distance * sin, 0);
        }
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
