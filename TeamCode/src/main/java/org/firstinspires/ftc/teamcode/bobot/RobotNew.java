package org.firstinspires.ftc.teamcode.bobot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Drive;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Flower;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.bobot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.bobot.util.PoseStorage;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class RobotNew {
    private static final RobotNew INSTANCE = new RobotNew();
    private static Pose defaultPose = new Pose(135, 9, Math.PI / 2);

    public final Drive drive = new Drive();
    public final Intake intake = new Intake();
    public final ShooterNew shooter = new ShooterNew();
    public final Limelight limelight = new Limelight();
    public final Turret turret = new Turret();
    public final Flower flower = new Flower();

    private RobotNew() {}

    public static RobotNew getInstance() {
        return INSTANCE;
    }

    public void init(HardwareMap hw) {
        init(hw, PoseStorage.getOrDefault(defaultPose));
    }

    public void init(HardwareMap hw, Pose startPose) {
        drive.init(hw, startPose);
        intake.init(hw);
        shooter.init(hw);
        shooter.turnOff();
        turret.init(hw, limelight);
        turret.off();
        flower.init(hw);
        flower.closeFlower();
        limelight.init(hw);
    }
    public void turnOn() {
        shooter.turnOn();
        turret.on();
    }

    public void turnOff() {
        intake.spinOff();
        shooter.turnOff();
        turret.off();
        limelight.stop();
        drive.drive(0.0, 0.0, 0.0);
        drive.periodic();
        flower.closeFlower();
    }

    public void periodic() {
        drive.periodic();
        intake.periodic();
        shooter.periodic();
        limelight.periodic();
        turret.periodic();
    }

    public void saveEnd() {
        PoseStorage.save(drive.getPose());
    }

    @Config
    public static class ShooterNew extends SubsystemBase {

        private static final double STANDBY_SPEED_FRACTION = 0.0;

        public DcMotorEx l, r;
        private Servo stopper;
        private VoltageSensor voltageSensor;

        public static boolean predictiveCompEnabled = true;
        public static double shotBoostTPS = 120;
        public static double shotBoostDuration = 1;
        public static double shotBoostDelay = 0;
        public void anticipateShot() {
            if (!on || getCommandedTarget() <= 0) return;

            lastShotTime = System.nanoTime();
        }
        private long lastShotTime = -1;
        public static double blockedPosition = 0.35, feedingPosition = 0.5;
        public static double kP = 0.072, kI = 0.0, kD = 0.001;
        public static double kS = 0.08, kV = 0.00037;
        public static double baseVel = 1234, offsetTuner = 0;
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
            // Reset any final-shot override left by the previous OpMode.
            baseVel = 1234;
            // Readiness must wait for an encoder sample from this OpMode.
            currentVelocity = Double.NaN;
            acceleration = 0;
            lastVelocity = 0;
            l = new CachingDcMotorEx(hw.get(DcMotorEx.class, "shooter1"));
            r = new CachingDcMotorEx(hw.get(DcMotorEx.class, "shooter2"));

            r.setDirection(DcMotorSimple.Direction.REVERSE);

            l.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            r.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            voltageSensor = hw.voltageSensor.iterator().next();

            stopper = hw.get(Servo.class, "stopper");
            setFeeding(false);

            lastTime = 0;
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

        public double getActiveTargetTPS() {
            return previousTarget;
        }

        private double getShotBoostTPS(long currentTime) {
            double timeSinceShot = lastShotTime < 0
                    ? -1
                    : (currentTime - lastShotTime) * 1e-9;

            boolean boosting = predictiveCompEnabled
                    && Double.isFinite(shotBoostTPS)
                    && shotBoostTPS > 0
                    && Double.isFinite(shotBoostDelay)
                    && shotBoostDelay >= 0
                    && Double.isFinite(shotBoostDuration)
                    && shotBoostDuration > 0
                    && timeSinceShot >= shotBoostDelay
                    && timeSinceShot < shotBoostDelay + shotBoostDuration;

            return boosting ? shotBoostTPS : 0;
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
            if (!feeding) lastShotTime = -1;
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
            double dt = lastTime == 0 ? 0 : (currentTime - lastTime) * 1e-9;
            lastTime = currentTime;

            currentVelocity = Math.abs(l.getVelocity());

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

            target += getShotBoostTPS(currentTime);
            boolean targetChanged = target != previousTarget;
            if(targetChanged){
                resetPID();
                previousTarget = target;
            }

            double error = target + offsetTuner - currentVelocity;

            if(validDt){
                integral += error * dt;
                integral = Math.max(-integralLimit, Math.min(integralLimit, integral));
            }

            double derivative = validDt && !targetChanged
                    ? (error - lastError) / dt : 0;

            lastError = error;

            double pidPower = kP * error + kI * integral + kD * derivative;
            double feedforwardPower = kS + kV * target;

            double batteryVoltage = volComp ? getVoltage() : nominalVoltage;

            if(!Double.isFinite(batteryVoltage) || batteryVoltage <= 1){
                set(0);
                return;
            }

            double output = (pidPower + feedforwardPower)
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
}
