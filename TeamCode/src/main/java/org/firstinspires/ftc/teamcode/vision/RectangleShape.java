package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;

public class RectangleShape {

    private double minx;
    private double miny;
    private double maxx;
    private double maxy;
    private double width;
    private double height;
    private double area;

    public RectangleShape() {
        super();
    }

    public RectangleShape setPoints(double minx, double miny, double maxx, double mxay) {
        this.minx = minx;
        this.miny = miny;
        this.maxx = maxx;
        this.maxy = mxay;
        this.width = maxx - minx;
        this.height = maxy - miny;
        this.area = width * height;

        return this;
    }

    public RectangleShape setDims(double minx, double miny, double width, double height) {
        this.minx = minx;
        this.miny = miny;
        this.maxx = minx + width;
        this.maxy = miny + height;
        this.width = width;
        this.height = height;
        this.area = width * height;

        return this;
    }

    public double getMinX() {
        return minx;
    }

    public double getMinY() {
        return miny;
    }

    public double getMaxX() {
        return maxx;
    }

    public double getMaxY() {
        return maxy;
    }

    public double getWidth() {
        return width;
    }

    public double getHeight() {
        return height;
    }

    public double getArea() {
        return area;
    }

    public Point getCenter() {
        return new Point((maxx - minx) / 2.0, (maxy - miny) / 2.0);
    }
}
