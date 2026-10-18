package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Pinpoint Raw Test")
public class pinpointTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        GoBildaPinpointDriver odo =
                hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(
                2.636909785233145,
                -4.6039220854992005,
                DistanceUnit.INCH
        );

        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        odo.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {

            odo.update();

            telemetry.addData("X", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading", odo.getHeading(AngleUnit.RADIANS));
            telemetry.addData("X Vel", odo.getVelX(DistanceUnit.INCH));
            telemetry.addData("Y Vel", odo.getVelY(DistanceUnit.INCH));

            telemetry.update();
        }
    }
}