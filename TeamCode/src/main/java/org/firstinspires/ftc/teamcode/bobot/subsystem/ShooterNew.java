package org.firstinspires.ftc.teamcode.bobot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config
public class ShooterNew extends SubsystemBase {

    private static final double STANDBY_SPEED_FRACTION = 0.0;

    public ShooterNew(){}

    public DcMotorEx l, r;
    private Servo stopper;
    private VoltageSensor voltageSensor;

    public static boolean predictiveCompEnabled = false;
    public static double shotBoost = 0.054;
    public static double shotBoostDuration = 0.32;
    public static double shotBoostDelay = 0.235;
    public void anticipateShot() {
        if (!on || getCommandedTarget() <= 0) return;

        lastShotTime = System.nanoTime();
    }
    private long lastShotTime = -1;
    public double predictivePower = 0;
    public static double blockedPosition = 0.35, feedingPosition = 0.5;
    public static double kP = 0.001, kI = 0.0, kD = 0.0;
    public static double kS = 0.0433, kV = 0.00035;
    public static double baseVel = 2420, offsetTuner = 0;
    public static double nominalVoltage = 12.0;
    public static double integralLimit = 1000;

    public static boolean volComp = true;
    public static boolean on = true;

    private double previousTarget = 0;
    private double lastError = 0, integral = 0;
    private double lastVelocity = 0;
    private long lastTime = 0;
    private boolean standby = false;

    public double power = 0;
    public double currentVelocity = 0;
    public double acceleration = 0;

    public enum BlockStates {
        BLOCK, SUPPLY
    }

    private BlockStates blockState = BlockStates.BLOCK;

    public void init(HardwareMap hw){
        l = new CachingDcMotorEx(hw.get(DcMotorEx.class, "shooter1"));
        r = new CachingDcMotorEx(hw.get(DcMotorEx.class, "shooter2"));

        r.setDirection(DcMotorSimple.Direction.REVERSE);

        l.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        r.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hw.voltageSensor.iterator().next();

        stopper = hw.get(Servo.class, "stopper");
        setFeeding(false);

        lastTime = System.nanoTime();
    }

    public void set(double p){
        power = p;
        l.setPower(p);
        r.setPower(p);
    }

    public double getTarget(){
        return baseVel;
    }

    public void setStandby(boolean standby){
        this.standby = standby;
    }

    public double getCommandedTarget(){
        return standby ? getTarget() * STANDBY_SPEED_FRACTION : getTarget();
    }

    public double getVel(){
        return Math.abs(currentVelocity);
    }

    public double getAccel(){
        return acceleration;
    }

    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

    public double getShooter1_Amps(){
        return l.getCurrent(CurrentUnit.AMPS);
    }

    public double getShooter2_Amps(){
        return r.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isShooterAtTarget(){
        double target = getCommandedTarget();
        return on && target > 0 && Math.abs(getVel() - target) <= 10;
    }

    public boolean isShooterAlmostAtTarget(){
        double target = getCommandedTarget();
        return on && target > 0 && getVel() >= target - 300;
    }

    public void setFeeding(boolean feeding){
        blockState = feeding ? BlockStates.SUPPLY : BlockStates.BLOCK;
        stopper.setPosition(feeding ? feedingPosition : blockedPosition);
    }

    public void setSupplyState(BlockStates state){
        setFeeding(state == BlockStates.SUPPLY);
    }

    public BlockStates getSupplyState(){
        return blockState;
    }

    private void resetPID(){
        lastError = 0;
        integral = 0;
        previousTarget = 0;
    }

    public void turnOff(){
        resetPID();
        set(0);
        setFeeding(false);
        standby = false;
        on = false;
    }

    public void turnOn(){
        on = true;
    }

    public boolean isOn(){
        return on;
    }

    @Override
    public void periodic(){
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) * 1e-9;
        lastTime = currentTime;

        currentVelocity = Math.abs(r.getVelocity());

        boolean validDt = dt > 0 && dt < 0.25;

        acceleration = validDt
                ? (currentVelocity - lastVelocity) / dt : 0;

        lastVelocity = currentVelocity;

        double target = getCommandedTarget();

        if(!on || !Double.isFinite(target) || target <= 0 ||
                !Double.isFinite(currentVelocity)){
            resetPID();
            set(0);
            return;
        }

        if(target != previousTarget){
            resetPID();
            previousTarget = target;
        }

        double error = target + offsetTuner - currentVelocity;

        if(validDt){
            integral += error * dt;
            integral = Math.max(-integralLimit, Math.min(integralLimit, integral));
        }

        double derivative = validDt
                ? (error - lastError) / dt : 0;

        lastError = error;

        double pidPower = kP * error + kI * integral + kD * derivative;
        double feedforwardPower = kS + kV * target;

        double timeSinceShot = lastShotTime < 0
                ? -1
                : (currentTime - lastShotTime) * 1e-9;

        boolean boosting = predictiveCompEnabled
                && Double.isFinite(shotBoost)
                && shotBoost >= 0
                && Double.isFinite(shotBoostDelay)
                && shotBoostDelay >= 0
                && Double.isFinite(shotBoostDuration)
                && shotBoostDuration > 0
                && timeSinceShot >= shotBoostDelay
                && timeSinceShot < shotBoostDelay + shotBoostDuration;

        predictivePower = boosting ? shotBoost : 0;

        double batteryVoltage = volComp ? getVoltage() : nominalVoltage;

        if(!Double.isFinite(batteryVoltage) || batteryVoltage <= 1){
            set(0);
            return;
        }

        double output = (pidPower + feedforwardPower + predictivePower)
                * nominalVoltage / batteryVoltage;

        set(Double.isFinite(output) ? Math.max(0, Math.min(1, output)) : 0);
    }

    public InstantCommand on(){
        return new InstantCommand(this::turnOn);
    }

    public InstantCommand off(){
        return new InstantCommand(this::turnOff);
    }
}
