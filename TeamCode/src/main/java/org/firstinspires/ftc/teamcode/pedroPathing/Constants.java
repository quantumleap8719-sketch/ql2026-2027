package org.firstinspires.ftc.teamcode.pedroPathing;

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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_right");
        c.backRightName.set("back_left");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.6879951296828875);
        c.yPodOffset.set(4.081831504040816);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3589707139395332);
                Controller secondaryTranslationalForward = Controller.proportional(0.132630086926525);
                Controller primaryTranslationalLateral = Controller.proportional(0.5009335027171506);
                Controller secondaryTranslationalLateral = Controller.proportional(0.185081544064276);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014377232075645622));
                c.brake.set(Controller.proportionalFeedforward(0.012220647264298778));

                c.headingFeedback.set(Controller.proportional(6.336302341640145));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(-0.019114823147895792, 0.02709537753143227));

                c.linearBrakeCoefficients.set(Matrix.diag(0.16080089357912858, 0.047939824069321034));
                c.quadraticBrakeCoefficients.set(Matrix.diag(4.158353014363504E-4, 0.0021821322287369814));

                c.maxAchievableForwardVelocity.set(59.43690033343048);
                c.maxAchievableStrafeVelocity.set(44.981680489710755);
                c.naturalForwardDeceleration.set(27.30483840808096);
                c.naturalStrafeDeceleration.set(61.90193793210256);
            }
    );
}