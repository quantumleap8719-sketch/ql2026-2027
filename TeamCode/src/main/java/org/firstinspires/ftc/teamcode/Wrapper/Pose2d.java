package org.firstinspires.ftc.teamcode.Wrapper;

import androidx.annotation.NonNull;
import java.util.Locale;

public final class Pose2d {
    private double x,y,heading; //(in,in,rad)

    public Pose2d(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double x() { return x; }
    public double y() { return y; }
    public double heading() { return heading; }
    public double distTo(Pose2d target){
        return Math.sqrt(Math.pow(target.x-x,2)+Math.pow(target.y-y,2));
    }

    public void set(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(Pose2d other){
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US,"(%.3f, %.3f, %.3f)", x, y, (heading*180)/Math.PI);
    }
}