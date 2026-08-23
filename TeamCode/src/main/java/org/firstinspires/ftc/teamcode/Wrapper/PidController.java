package org.firstinspires.ftc.teamcode.Wrapper;

/**
 * Simple PID controller (no feedforward).
 *
 * - Time units: seconds (dt is in seconds).
 * - target is the desired position (x, y, heading, etc.).
 * - measured is the current position.
 *
 * Typical use for drivetrain:
 *   xController.target = targetPose.x;
 *   double xCmd = xController.update(currentPose.x, dt);
 *
 * You should tune kP, kI, kD empirically.
 */

public final class PidController {

    // Gains
    private double kP;
    private double kI;
    private double kD;

    // Public so you can inspect in telemetry if you want
    public double target;
    public double lastError;
    public double lastOutput;

    // Internal state
    private double integral;
    private double prevError;
    private boolean firstRun = true;

    // Integral clamping
    private boolean integralLimited = false;
    private double integralMin;
    private double integralMax;

    // Output clamping
    private boolean outputLimited = false;
    private double outputMin;
    private double outputMax;

    // Optional input wrapping (e.g., angles) for target/measurement mode
    private boolean inputBounded = false;
    private double minInput;
    private double maxInput;

    public PidController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setGains(double kP, double kI, double kD) {
        setCoefficients(kP, kI, kD);
    }

    public void setIntegralLimits(double min, double max) {
        if (min < max) {
            integralLimited = true;
            integralMin = min;
            integralMax = max;
        }
    }

    public void setOutputLimits(double min, double max) {
        if (min < max) {
            outputLimited = true;
            outputMin = min;
            outputMax = max;
        }
    }

    /**
     * Enable wrapping on the input space (useful for angles).
     *
     * Example for headings in radians: setInputBounds(-Math.PI, Math.PI).
     * The controller will choose the shortest error around the circle
     * when using update(target, measured, dt) or update(measured, dt).
     */
    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        lastError = 0.0;
        lastOutput = 0.0;
        firstRun = true;
    }

    /**
     * Core update using stored target (target - measured).
     */
    public double update(double measured, double dtSeconds) {
        if (dtSeconds <= 0) {
            return lastOutput;
        }

        double error = target - measured;

        // Optional wrapping (e.g., for angles)
        if (inputBounded) {
            double range = maxInput - minInput;
            while (Math.abs(error) > range / 2.0) {
                error -= Math.copySign(range, error);
            }
        }

        return updateFromError(error, dtSeconds);
    }

    /**
     * Convenience overload: set target then run controller.
     */
    public double update(double target, double measured, double dtSeconds) {
        this.target = target;
        return update(measured, dtSeconds);
    }

    /**
     * NEW: Direct error-based update.
     * Use this when you already computed an error (e.g., wrapped angle error).
     *
     * @param error     current error (setpoint - measurement), already in the units you want
     * @param dtSeconds loop time in seconds
     * @return control output
     */
    public double updateFromError(double error, double dtSeconds) {
        if (dtSeconds <= 0) {
            return lastOutput;
        }

        if (firstRun) {
            prevError = error;
            firstRun = false;
        }

        // Proportional
        double p = kP * error;

        // Integral
        integral += error * dtSeconds;
        if (integralLimited) {
            if (integral > integralMax) integral = integralMax;
            if (integral < integralMin) integral = integralMin;
        }
        double i = kI * integral;

        // Derivative
        double derivative = (error - prevError) / dtSeconds;
        double d = kD * derivative;

        prevError = error;
        lastError = error;

        double output = p + i + d;

        // Output clamp
        if (outputLimited) {
            if (output > outputMax) output = outputMax;
            if (output < outputMin) output = outputMin;
        }

        lastOutput = output;
        return output;
    }

    public double getkP() { return kP; }
    public double getkI() { return kI; }
    public double getkD() { return kD; }
    public double getIntegral() { return integral; }

}