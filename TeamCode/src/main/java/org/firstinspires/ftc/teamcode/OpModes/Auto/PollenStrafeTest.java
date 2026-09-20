package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
@Autonomous(name = "Pollen Strafe Test", group = "Testing")
public class PollenStrafeTest extends LinearOpMode {
    Mecanum mecanum;
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        mecanum = new Mecanum(hardwareMap, telemetry);
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
                double tx = result.getTx();
                telemetry.addData("Pollen", "found");
                telemetry.addData("Area", area);
                if(Math.abs(tx) < 3.8){
                    mecanum.setPower(0,0,0);
                }

                // If pollen is still far away
                if (area < 8) {
                    // Strafe toward the Limelight
                    mecanum.setPower(-0.25, 0, 0);
                    if (tx > 4) {
                        mecanum.setPower(-.25, .25, 0);
                    }// Pollen to left
                    else if (tx < -4) {
                        mecanum.setPower(-.25, -.25, 0);
                    }
                } else {
                    mecanum.setPower(0, 0, 0);
                    telemetry.addLine("pollen reached");
                }
                // Pollen is close enoughelse {

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