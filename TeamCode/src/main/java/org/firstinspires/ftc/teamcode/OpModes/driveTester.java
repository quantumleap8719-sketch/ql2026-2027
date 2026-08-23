package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
import org.firstinspires.ftc.teamcode.Wrapper.Pose2d;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;


@TeleOp(name="Drive Tester", group="Testing")
public class driveTester extends LinearOpMode {
    Mecanum mecanum;
    GoBildaPinpointDriver odo;
    boolean fieldCentric = false;
    TelemetryManager telemetryM;
    GamepadEx gamepadEx1;
    @Override
    public void runOpMode() throws InterruptedException {
        mecanum = new Mecanum(hardwareMap,telemetryM);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.init();
        odo.resetPosAndIMU();
        gamepadEx1 = new GamepadEx(gamepad1);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        waitForStart();


        while (opModeIsActive()) {
            gamepadEx1.loop();


            odo.update();

            if(gamepadEx1.gamepad.optionsWasPressed()){
                fieldCentric = !fieldCentric;
            }

            if(fieldCentric) {
                mecanum.driveCentric(gamepadEx1.gamepad, 1, 1, odo.getHeading());
            }else{
                mecanum.drive(gamepadEx1.gamepad,1,1);
            }

            mecanum.write();

            telemetryM.debug("Robot Pos",odo.getPosition());
            telemetryM.update();
        }
    }
}