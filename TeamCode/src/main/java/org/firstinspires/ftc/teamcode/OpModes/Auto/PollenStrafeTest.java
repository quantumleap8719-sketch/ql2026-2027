package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
@Autonomous(name="Pollen Strafe Test",group="Testing")
public class PollenStrafeTest extends LinearOpMode{
    Mecanum mecanum;
    Limelight3A limelight;
    TelemetryManager telemetryM;
    boolean aligned=false;
    @Override
    public void runOpMode() throws InterruptedException{
        mecanum = new Mecanum(hardwareMap,telemetry);
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        // green pipeline

        limelight.pipelineSwitch(1);
        limelight.start();
        telemetryM.addLine("Ready");
        telemetryM.update();

        waitForStart();

        while(opModeIsActive()){
            LLResult result=limelight.getLatestResult();
            if(result!=null &&result.isValid()){
                double area=result.getTa();
                double tx=result.getTx();
                telemetryM.addData("Pollen","found");
                telemetryM.addData("TX",tx);
                telemetryM.addData("Area",area);
                // close enough
                if(area>=8){
                    mecanum.setPower(0,0,0);
                    telemetryM.addLine("Pollen reached");
                }
                // first line up with ball
                else if(!aligned){
                    if(tx>4){
                        mecanum.setPower(0,0.25,0);
                        telemetryM.addLine("Moving forward");
                    }
                    else if(tx<-4){
                        mecanum.setPower(0,-0.25,0);
                        telemetryM.addLine("Moving backward");
                    }
                    else{
                        // lined up, stop for a sec
                        mecanum.setPower(0,0,0);
                        mecanum.writeAll();
                        telemetryM.addLine("Aligned");
                        telemetryM.update();
                        sleep(1000);
                        aligned=true;
                    }
                }
                // now only move toward ball
                else{
                    mecanum.setPower(-0.25,0,0);
                    telemetryM.addLine("Moving toward pollen");
                }
            }
            else{
                // no ball
                mecanum.setPower(0,0,0);
                telemetryM.addLine("No pollen");
            }
            mecanum.writeAll();
            telemetryM.update();
        }
        mecanum.setPower(0,0,0);
        mecanum.writeAll();
        limelight.stop();
    }
}