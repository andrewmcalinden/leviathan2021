package org.firstinspires.ftc.teamcode.math;

public class Vector {
    public double x;
    public double y;
    public double magnitude;
    public double angle;

    public Vector(double x1, double y1){
        x = x1;
        y = y1;
        magnitude = Math.hypot(x1, y1);
        angle = Math.atan2(y1, x1);
    }

    public Vector rotated(double radians){
        double newX = x * Math.cos(radians) - y * Math.sin(radians);
        double newY = x * Math.sin(radians) + y * Math.cos(radians);
        return new Vector(newX, newY);
    }
}
