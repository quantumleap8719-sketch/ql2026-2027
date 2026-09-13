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
        // Pipeline 0 should be your yellow detection pipeline
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                telemetry.addData("TX", tx);
                if (tx > 2) {
                    mecanum.setPower(0.25, 0, 0);
                }// Pollen to left
                else if (tx < -2) {
                    mecanum.setPower(-0.25, 0, 0);
                }

                // Pollen in center
                else {
                    mecanum.setPower(0, 0, 0);
                }

            } else {
                // No pollen detected
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