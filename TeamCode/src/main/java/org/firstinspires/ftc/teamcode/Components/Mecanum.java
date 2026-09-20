package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.PidController;
import org.firstinspires.ftc.teamcode.Wrapper.Pose2d;
import org.firstinspires.ftc.teamcode.Wrapper.CacheMotor;



public class Mecanum{
    CacheMotor[] motors = new CacheMotor[4];

Telemetry telemetry;
    PidController xPID, yPID, hPID;
    public static double kp = 0.08;
    public static double ki = 0;
    public static double kd = 0.01;

    public static double kph = 2.5;
    public static double kih = 0;
    public static double kdh = 0.12;
    int counter;

    public Mecanum(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        motors[0] = new CacheMotor(map, "front_left");
        motors[1] = new CacheMotor(map, "front_right");
        motors[2] = new CacheMotor(map, "back_left");
        motors[3] = new CacheMotor(map, "back_right");

        motors[0].getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        xPID = new PidController(kp, ki, kd);
        yPID = new PidController(kp, ki, kd);
        hPID = new PidController(kph, kih, kdh);

        counter = 0;

        motors[0].getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        motors[2].getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setCoast(){
        motors[0].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors[1].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors[2].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors[3].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrake(){
        motors[0].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void write(){
        motors[counter].write();
        counter = (counter + 1) % 4;
    }
    public void writeAll() {
        for (int i = 0; i < 4; i++) {
            motors[i].write();
        }
    }


    public void setPower(double x, double y, double rot) {
        // Compute raw motor powers
        double frontLeft  = -y - x + rot;
        double frontRight = -y + x - rot;
        double backLeft   = -y + x + rot;
        double backRight  = -y - x - rot;

        // Find max magnitude
        double max = Math.max(
                Math.max(Math.abs(frontLeft),  Math.abs(frontRight)),
                Math.max(Math.abs(backLeft),   Math.abs(backRight))
        );

        // Normalize if needed
        if (max > 1.0) {
            frontLeft  /= max;
            frontRight /= max;
            backLeft   /= max;
            backRight  /= max;
        }

        // Apply powers (assuming index ordering matches)
        motors[0].setPower(frontLeft);
        motors[1].setPower(frontRight);
        motors[2].setPower(backLeft);
        motors[3].setPower(backRight);
    }

    public void setPowerCentric(double xField, double yField, double rot, double heading) {
        double c = Math.cos(heading);
        double s = Math.sin(heading);


        double xRobot =  xField * s - yField * c;
        double yRobot =  xField * c + yField * s;


        setPower(xRobot, yRobot, rot);
    }



    public void drive(Gamepad gamepad, double scale, double turnScale, double maxMove, double maxTurn){
        setPower(Range.clip(gamepad.left_stick_x * scale, -maxMove, maxMove), Range.clip(-gamepad.left_stick_y * scale, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x * turnScale, -maxTurn, maxTurn));
    }

    public void drive(Gamepad gamepad, double maxMove, double maxTurn){
        setPower(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(-gamepad.left_stick_y, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x, -maxTurn, maxTurn));
    }

    public void driveCentric(Gamepad gamepad, double maxMove, double maxTurn, double heading){
        setPowerCentric(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(-gamepad.left_stick_y, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x, -maxTurn, maxTurn), heading);
    }

    public void driveCentric(Gamepad gamepad, double turnScale, double maxMove, double maxTurn, double heading){
        setPowerCentric(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(-gamepad.left_stick_y, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x * turnScale, -maxTurn, maxTurn), heading);
    }

    public static double shortestAngle(double target, double current) {
        double diff = target - current;
        return Math.atan2(Math.sin(diff), Math.cos(diff)); // (-π, π]
    }

    public void goToPoint(Pose2d tPos, Pose2d Pos, double dt, double xspeed, double yspeed, double zspeed){
        xPID.setOutputLimits(-xspeed, xspeed);
        yPID.setOutputLimits(-yspeed, yspeed);
        hPID.setOutputLimits(-zspeed, zspeed);

        double hError = shortestAngle(tPos.heading(), Pos.heading());

        telemetry.addData("Target Pos: ", tPos);
        telemetry.addData("Current Pos: ", Pos);

        double xPower = xPID.update(tPos.x(), Pos.x(), dt);
        double yPower = yPID.update(tPos.y(), Pos.y(), dt);
        double hPower = hPID.updateFromError(hError, dt);

        setPowerCentric(xPower,yPower,hPower, Pos.heading());

        //  telemetry.addData("xPower",xPower);
        //  telemetry.addData("yPower",yPower);
        //  telemetry.addData("hPower",hPower);
    }

    public void setTransPid(double kp, double ki, double kd) {
        xPID.setGains(kp, ki, kd);
        yPID.setGains(kp, ki, kd);
    }

    public void setHeadingPid(double kp, double ki, double kd) {
        hPID.setGains(kp, ki, kd);
    }

    public void resetPids() {
        xPID.reset();
        yPID.reset();
        hPID.reset();
    }
}