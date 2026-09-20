package org.firstinspires.ftc.teamcode.OpModes;


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
    GamepadEx gamepadEx1;
    @Override
    public void runOpMode() throws InterruptedException {
        mecanum = new Mecanum(hardwareMap,telemetry);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.init();
        odo.resetPosAndIMU();
        gamepadEx1 = new GamepadEx(gamepad1);


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


        }
    }
}