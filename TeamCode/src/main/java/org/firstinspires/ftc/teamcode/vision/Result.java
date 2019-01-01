package org.firstinspires.ftc.teamcode.vision;


import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;

public class Result {

    private CircleShape circle;
    private RotatedRect rectangle;
    private MatOfPoint contour;
    private FitShape shape;
    private RectangleShape frameSize;

    public Result() {
        super();
        this.circle = null;
        this.rectangle = null;
        this.contour = null;
        this.shape = null;
        this.frameSize = null;
    }

    public RectangleShape getFrameSize() {
        return frameSize;
    }
    public void setFrameSize(RectangleShape frameSize) {
        this.frameSize = frameSize;
    }
    public FitShape getShape() {
        return shape;
    }
    public void setShape(FitShape shape) {
        this.shape = shape;
    }
    public CircleShape getCircle() {
        return circle;
    }
    public void setCircle(CircleShape circle) {
        this.circle = circle;
    }
    public RotatedRect getRectangle() {
        return rectangle;
    }
    public void setRectangle(RotatedRect rectangle) {
        this.rectangle = rectangle;
    }
    public MatOfPoint getContour() {
        return contour;
    }
    public void setContour(MatOfPoint contour) {
        this.contour = contour;
    }
    public Point getCenter() {
        Point center = null;

        switch(shape) {
            case RECTANGLE:
                center = getRectangle().center;
                break;
            case CIRCLE:
                center = getCircle().getCenter();
                break;
        }

        return center;
    }
    public double getArea() {
        double area = 0;

        switch(shape) {
            case CIRCLE:
                area = getCircle().getArea();
                break;
            case RECTANGLE:
                Size s = getRectangle().size;
                area = s.height * s.width;
                break;
        }

        return area;
    }
    public static RectangleShape getMBR(RotatedRect rr) {
        RectangleShape r = new RectangleShape();

        Rect cvr = rr.boundingRect();
        r.setDims(cvr.x, cvr.y, cvr.width, cvr.height);

        return r;
    }
}