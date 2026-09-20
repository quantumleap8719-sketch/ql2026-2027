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


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        limelight.start();


        waitForStart();

        while (opModeIsActive()) {

            odo.update();

            double heading = odo.getHeading(AngleUnit.RADIANS);

            limelight.updateRobotOrientation(heading);

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();


                for (LLResultTypes.FiducialResult fiducial : fiducials) {

                    int id = fiducial.getFiducialId();

                    double tx = fiducial.getTargetXDegrees();
                    double ty = fiducial.getTargetYDegrees();

                    Pose3D pose = fiducial.getTargetPoseCameraSpace();

                    double x = pose.getPosition().x;
                    double y = pose.getPosition().y;
                    double z = pose.getPosition().z;

                    double distanceMeters = Math.sqrt(x * x + y * y + z * z);

                    double distanceInches = distanceMeters * 39.3701;


                }

            } else {
            }


            sleep(20);
        }

        limelight.stop();
    }
}