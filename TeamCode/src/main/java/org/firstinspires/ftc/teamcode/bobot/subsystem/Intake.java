package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake extends SubsystemBase {
    private MotorEx intake;
    private MotorEx transfer;
    private boolean transferCutoff;
    private boolean outtaking;
    public static double intakeShoot = 0.36, transShoot = 0.88;
    public static double transferCurrentLimitMilliamps = 6500.0;

    public void init(HardwareMap hw) {
        intake = new MotorEx(hw, "intake");
        transfer = new MotorEx(hw, "trans");
        intake.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        intake.setInverted(true);
        transfer.setInverted(false);
        intake.stopMotor();
        transfer.stopMotor();
        transferCutoff = false;
        outtaking = false;
        intake.setCachingTolerance(0.05);
        transfer.setCachingTolerance(0.05);
    }

    private void setPower(double intakePower, double transferPower) {
        setPower(intakePower, transferPower, false);
    }

    private void setPower(double intakePower, double transferPower, boolean outtaking) {
        this.outtaking = outtaking;
        intake.set(-intakePower);
        if (transferPower == 0.0) transferCutoff = false;
        transfer.set(transferCutoff && !outtaking ? 0.0 : transferPower);
    }

    @Override
    public void periodic() {
        if (!outtaking && transfer != null && transfer.get() != 0.0
                && transfer.getCurrent(CurrentUnit.MILLIAMPS) > transferCurrentLimitMilliamps) {
            transferCutoff = true;
            transfer.set(0.0);
        }
    }

    public void spinIn() {
        setPower(1.0, 1.0);
    }

    public double getTransferCurrentMilliamps() {
        return transfer.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public void spinOut() {
        setPower(-0.8, -0.8, true);
    }

    public void spinOff() {
        setPower(0.0, 0.0);
    }

    public void spinShoot() {
        setPower(1.0, 1.0);
    }
    public void beforeShoot() { setPower(-0.25, -0.3); }

    public void spinSlowShoot() {
        setPower(intakeShoot, transShoot);
    }

    public InstantCommand in() {
        return new InstantCommand(this::spinIn, this);
    }

    public InstantCommand out() {
        return new InstantCommand(this::spinOut, this);
    }

    public InstantCommand off() {
        return new InstantCommand(this::spinOff, this);
    }

    public InstantCommand shoot() {
        return new InstantCommand(this::spinShoot, this);
    }

    public InstantCommand slowShoot() {
        return new InstantCommand(this::spinSlowShoot, this);
    }
}
