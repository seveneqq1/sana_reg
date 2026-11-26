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
                Controller primaryTranslationalForward = Controller.proportional(0.25313763194380434);
                Controller secondaryTranslationalForward = Controller.proportional(0.0935275910411365);
                Controller primaryTranslationalLateral = Controller.proportional(0.4062719400695573);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1501066260695289);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009993921362254691));
                c.brake.set(Controller.proportionalFeedforward(0.008494833157916487));

                c.headingFeedback.set(Controller.proportional(3.8351214511975584));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07497193082559139, 0.0042878597720331455));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1382723081214415, 0.07967512729889917));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012408111630435687, 0.0018659059776458833));

                c.maxAchievableForwardVelocity.set(84.44752303478704);
                c.maxAchievableStrafeVelocity.set(69.18154651049821);
                c.naturalForwardDeceleration.set(20.622328395529607);
                c.naturalStrafeDeceleration.set(56.68631720923416);
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
