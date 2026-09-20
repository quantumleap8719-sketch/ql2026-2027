package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.Mecanum;

@Autonomous(name="Pollen Align Test",group="Testing")
public class PollenStrafeTest extends LinearOpMode{
    Mecanum mecanum;
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException{
        mecanum=new Mecanum(hardwareMap,telemetry);
        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        // green ball
        limelight.pipelineSwitch(1);
        limelight.start();
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            LLResult result=limelight.getLatestResult();
            if(result!=null&&result.isValid()){
                double tx=result.getTx();
                telemetry.addData("Pollen","found");
                telemetry.addData("TX",tx);
                // ball one side, move forward
                if(tx>1){
                    mecanum.setPower(0,0.25,0);
                    telemetry.addLine("Moving forward");
                }
                // ball other side, move backward
                else if(tx<-1){
                    mecanum.setPower(0,-0.25,0);
                    telemetry.addLine("Moving backward");
                }
                // camera lined up with ball
                else{
                    mecanum.setPower(0,0,0);
                    telemetry.addLine("aligned");
                }
            }
            else{
                mecanum.setPower(0,0,0);
                telemetry.addLine("No pollen");
            }
            mecanum.writeAll();
            telemetry.update();
        }
        mecanum.setPower(0,0,0);
        mecanum.writeAll();
        limelight.stop();
    }
}