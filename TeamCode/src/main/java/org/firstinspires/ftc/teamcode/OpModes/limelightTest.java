package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class limelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(4);


        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");


        limelight.start();

        telemetry.addLine("Limelight + Pinpoint ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            odo.update();

            // Get Pinpoint IMU heading
            double heading = odo.getHeading(AngleUnit.RADIANS);

            // Give Pinpoint heading to Limelight
            limelight.updateRobotOrientation(heading);

            // Get Limelight result
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                telemetry.addData("Tags Detected", fiducials.size());

                for (LLResultTypes.FiducialResult fiducial : fiducials) {

                    int id = fiducial.getFiducialId();

                    double tx = fiducial.getTargetXDegrees();
                    double ty = fiducial.getTargetYDegrees();

                    // AprilTag pose relative to camera
                    Pose3D pose = fiducial.getTargetPoseCameraSpace();

                    double x = pose.getPosition().x;
                    double y = pose.getPosition().y;
                    double z = pose.getPosition().z;

                    // 3D distance from camera to AprilTag
                    double distanceMeters = Math.sqrt(x * x + y * y + z * z);

                    double distanceInches = distanceMeters * 39.3701;

                    telemetry.addData("Tag ID", id);
                    telemetry.addData("TX", "%.2f°", tx);
                    telemetry.addData("TY", "%.2f°", ty);
                    telemetry.addData("Pinpoint Heading", "%.2f°", Math.toDegrees(heading));
                    telemetry.addData("Tag X", "%.3f m", x);
                    telemetry.addData("Tag Y", "%.3f m", y);
                    telemetry.addData("Tag Z", "%.3f m", z);
                    telemetry.addData("Distance", "%.2f in", distanceInches);
                }

            } else {
                telemetry.addLine("No valid AprilTag");
            }

            telemetry.update();

            sleep(20);
        }

        limelight.stop();
    }
}