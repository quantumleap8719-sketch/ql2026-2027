package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Components.GoBildaPinpointDriver;


@TeleOp(name = "Limelight MegaTag1 Localization")
public class limelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver odo;
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Limelight
        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        // Initialize GoBilda Pinpoint
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");


        // Reset Pinpoint position and calibrate its IMU
        odo.init();

        odo.resetPosAndIMU();

        // Select the configured AprilTag pipeline
        limelight.pipelineSwitch(4);

        telemetryM.addData("Status", "Initialized");
        telemetryM.addData("Localization", "MegaTag 1");
        telemetryM.update(telemetry);

        waitForStart();

        if (isStopRequested()) return;

        limelight.start();
        while (opModeIsActive()) {

            // Update Pinpoint localization
            odo.update();

            // Read Pinpoint heading in degrees
            double yaw = Math.toDegrees(odo.getHeading());

            // Send Pinpoint heading to Limelight for MegaTag 1
            limelight.updateRobotOrientation(yaw);

            // Retrieve latest Limelight result
            LLResult result = limelight.getLatestResult();

            telemetryM.addData("Pinpoint Yaw (deg)", yaw);

            if (result != null && result.isValid()) {

                Pose3D botpose = result.getBotpose();

                if (botpose != null) {

                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;

                    double limelightYaw = botpose.getOrientation()
                            .getYaw(AngleUnit.DEGREES);

                    telemetryM.addData("Status", "Valid pose");
                    telemetryM.addData("MegaTag 1 X (m)", x);
                    telemetryM.addData("MegaTag 1 Y (m)", y);
                    telemetryM.addData(
                            "MegaTag 1 Yaw (deg)", limelightYaw);

                    telemetryM.addData(
                            "AprilTags Used",
                            result.getBotposeTagCount());

                    telemetryM.addData(
                            "Average Tag Distance (m)",
                            result.getBotposeAvgDist());

                    telemetryM.addData(
                            "Average Tag Area (%)",
                            result.getBotposeAvgArea());

                    telemetryM.addData(
                            "Pipeline Index", result.getPipelineIndex());
                    telemetryM.addData("Actual Pipeline", result.getPipelineIndex());
                    telemetryM.addData("Pipeline Type", result.getPipelineType());
                    telemetryM.addData("Tags Used", result.getBotposeTagCount());

                } else {
                    telemetryM.addData(
                            "Status", "Field pose unavailable");
                }

            } else {
                telemetryM.addData(
                        "Status", "No valid Limelight result");
            }

            telemetryM.update(telemetry);
        }

        limelight.stop();
    }
}