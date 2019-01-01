package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;

public class CircleShape {

    private static final Point DEF_CENT = new Point(0, 0);
    private Point center;
    private double rad;

    public CircleShape() {
        this(DEF_CENT, 0);
    }

    public CircleShape(Point center, double rad) {
        super();
        this.center = center;
        this.rad = rad;
    }

    public Point getCenter() {
        return center;
    }

    public void setCenter(Point center) {
        this.center = center;
    }

    public double getRadius() {
        return rad;
    }

    public void setRadius(double rad) {
        this.rad = rad;
    }

    public double getArea() {
        return 2 * Math.PI * Math.pow(rad, 2);
    }

    public RectangleShape getMBR() {
        RectangleShape r = new RectangleShape();
        r.setDims(center.x - rad, center.y - rad, 2 * rad, 2 * rad);
        return r;
    }
}
