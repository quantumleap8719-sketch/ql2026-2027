package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class limelightTest extends LinearOpMode {

    private Limelight3A limelight;
    TelemetryManager telemetryM;
    GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(4);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        waitForStart();

        limelight.start();

        while(opModeIsActive()){
//            limelight.updateRobotOrientation(odo.getYawScalar());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                Pose3D botpose = llResult.getBotpose_MT2();
                telemetryM.debug("tx", llResult.getTx());
                telemetryM.debug("ty",llResult.getTy());
                telemetryM.debug("ta",llResult.getTa());
                telemetryM.debug("botpose",botpose);

            }
            telemetryM.debug("Telemetry is active");

        }
    }
}
