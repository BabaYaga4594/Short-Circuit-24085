package org.firstinspires.ftc.teamcode;

public class XyhVector {
    public double x;
    public double y;
    public double h;

    public XyhVector(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public XyhVector(XyhVector other) {
        this.x = other.x;
        this.y = other.y;
        this.h = other.h;
    }

    // Normalize the heading to be in the range of [-pi, pi)
    public double normDiff(double angle) {
        while (angle >= Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}