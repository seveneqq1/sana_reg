package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.083);
        c.yPodOffset.set(-6.52);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static MecanumConfig driveConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontleft");
        c.frontRightName.set("frontright");
        c.backLeftName.set("backleft");
        c.backRightName.set("backright");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

        c.manualBrakeMode.set(true);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2713417451470127);
                Controller secondaryTranslationalForward = Controller.proportional(0.10025352444685862);
                Controller primaryTranslationalLateral = Controller.proportional(0.4343597102137209);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16048430661875657);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010208064338392664));
                c.brake.set(Controller.proportionalFeedforward(0.008676854687633764));

                c.headingFeedback.set(Controller.proportional(4.616840318518724));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07161785357930975, 0.005503526805034083));

                c.linearBrakeCoefficients.set(Matrix.diag(0.13120279096918813, 0.08326427747461067));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001674222484949016, 0.0018657571890654263));

                c.maxAchievableForwardVelocity.set(82.98835843001606);
                c.maxAchievableStrafeVelocity.set(63.97471474755124);
                c.naturalForwardDeceleration.set(22.242147249095723);
                c.naturalStrafeDeceleration.set(61.97137318649415);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }
}
