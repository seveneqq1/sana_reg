package org.firstinspires.ftc.teamcode.bobot.auto.commands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.bobot.RobotNew;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;

@Config
public class RobotNewCommands {
    public static double readyTimeoutMs = 2000;
    public static double shooterTolerance = 120;
    public static double stopperOpenDelayMs = 150;
    public static double feedKickTimeMs = 0;
    public static double shootTimeMs = 2400;

    private RobotNewCommands() {}

    public static Command intake(RobotNew robot) {
        return new InstantCommand(() -> {
            robot.intake.spinIn();
            robot.rgb.intake();
        }, robot.intake);
    }

    public static Command nudgee(RobotNew robot, double radians) {
        return new InstantCommand(() -> robot.turret.nudge(-radians), robot.turret);
    }

    public static Command closeFlower(RobotNew robot) {
        return new InstantCommand(robot.flower::closeFlower, robot.flower);
    }

    public static Command intakeFlower(RobotNew robot) {
        return new InstantCommand(robot.flower::openFlower, robot.flower);
    }

    public static Command spinUp(RobotNew robot) {
        return new InstantCommand(() -> {
            robot.shooter.setStandby(false);
            robot.shooter.turnOn();
        }, robot.shooter);
    }

    public static Command setBaseVel(RobotNew robot, double velocityTPS) {
        return new InstantCommand(() -> {
            RobotNew.ShooterNew.baseVel = velocityTPS;
            robot.shooter.clearTargetTPS();
        }, robot.shooter);
    }

    public static Command useDistanceTPS(RobotNew robot) {
        return new InstantCommand(() -> robot.shooter.setDistanceTarget(() -> {
            Pose pose = robot.drive.getPose();
            return pose.distance(Turret.selectWestFieldTarget(pose));
        }), robot.shooter);
    }

    public static Command idle(RobotNew robot) {
        return new InstantCommand(() -> {
            robot.intake.spinOff();
            robot.shooter.turnOff();
            robot.rgb.idle();
        }, robot.intake, robot.shooter);
    }

    public static Command shoot(RobotNew robot) {
        return shoot(robot, shootTimeMs);
    }

    public static Command shoot(RobotNew robot, double shootMs) {
        return new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean stopperOpen;
            private boolean compensationTriggered;
            private boolean completedShot;
            private boolean done;

            {
                addRequirements(robot.shooter, robot.intake);
            }

            @Override
            public void initialize() {
                robot.rgb.shoot();
                stopperOpen = false;
                compensationTriggered = false;
                completedShot = false;
                done = false;
                robot.intake.spinOff();
                robot.shooter.setFeeding(false);
                robot.shooter.setStandby(false);
                robot.shooter.turnOn();
                timer.reset();
            }

            @Override
            public void execute() {
                if (!stopperOpen) {
                    double target = robot.shooter.getTarget();
                    double velocity = Math.abs(robot.shooter.getVel());
                    if (timer.milliseconds() >= readyTimeoutMs) {
                        done = true;
                    } else if (target > 0 && Math.abs(target - velocity) < shooterTolerance) {
                        robot.shooter.setFeeding(true);
                        stopperOpen = true;
                        timer.reset();
                    }
                }

                if (!stopperOpen || done) return;

                double elapsed = timer.milliseconds();
                if (elapsed >= shootMs) {
                    completedShot = true;
                    done = true;
                } else if (elapsed < stopperOpenDelayMs) {
                    robot.intake.spinOff();
                } else {
                    if (!compensationTriggered) {
                        robot.shooter.anticipateShot();
                        compensationTriggered = true;
                    }
                    if (elapsed < stopperOpenDelayMs + feedKickTimeMs) {
                        robot.intake.spinShoot();
                    } else {
                        robot.intake.spinSlowShoot();
                    }
                }
            }

            @Override
            public boolean isFinished() {
                return done;
            }

            @Override
            public void end(boolean interrupted) {
                robot.shooter.setFeeding(false);
                if (!interrupted && completedShot) {
                    robot.intake.spinIn();
                    robot.rgb.intake();
                } else {
                    robot.intake.spinOff();
                    robot.rgb.idle();
                }
            }
        };
    }
}
