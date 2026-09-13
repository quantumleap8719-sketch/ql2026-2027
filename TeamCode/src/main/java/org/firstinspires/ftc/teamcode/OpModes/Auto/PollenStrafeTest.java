package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
@Autonomous(name = "Pollen Strafe Test", group = "Testing")
public class PollenStrafeTest extends LinearOpMode {
    Mecanum mecanum;
    Limelight3A limelight;
    TelemetryManager telemetryM;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        mecanum = new Mecanum(hardwareMap, telemetryM);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Pipeline 1 = green pollen
        limelight.pipelineSwitch(1);
        limelight.start();
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double area = result.getTa();
                telemetry.addData("Pollen", "found");
                telemetry.addData("Area", area);

                // If pollen is still far away
                if (area < 8) {
                    // Strafe toward the Limelight
                    mecanum.setPower(0.25, 0, 0);
                }
                // Pollen is close enough
                else {
                    mecanum.setPower(0, 0, 0);
                    telemetry.addLine("pollen reached");
                }
            }
            else {
                // Don't see pollen
                mecanum.setPower(0, 0, 0);
                telemetry.addLine("No Pollen");
            }
            mecanum.writeAll();
            telemetry.update();
        }
        mecanum.setPower(0, 0, 0);
        mecanum.writeAll();
        limelight.stop();
    }
}