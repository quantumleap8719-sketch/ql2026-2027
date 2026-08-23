package org.firstinspires.ftc.teamcode.Wrapper;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class CacheMotor {

    private final DcMotorEx motor;
    private double lastPower = 0.0;
    private double requestedPower = 0.0;
    private boolean dirty = false;
    private static final double EPS2 = 0.00005625; //Eps = 0.0075, Eps^2 (EPS2)

    public CacheMotor(HardwareMap hw, String name) {
        motor = hw.get(DcMotorEx.class, name);
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    private static boolean significantChange(double a, double b) {
        double d = a - b;
        return (d * d) > EPS2;
    }

    public void setPower(double power) {
        if (significantChange(power, lastPower)) {
            requestedPower = power;
            dirty = true;
        }
    }

    public void write() {
        if (dirty) {
            motor.setPower(requestedPower);
            lastPower = requestedPower;
            dirty = false;
        }
    }
}