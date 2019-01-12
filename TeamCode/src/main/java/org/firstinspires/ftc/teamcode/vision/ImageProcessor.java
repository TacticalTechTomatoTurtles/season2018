package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ImageProcessor {

    public ImageProcessor() {
        super();
    }

    public List<Result> findCircles(Mat rgbImage, RectangleShape areaOfInterest, ColorRange colorRange, double minArea) {
        return findShapes(rgbImage, areaOfInterest, colorRange, minArea, FitShape.CIRCLE);
    }

    public List<Result> findRectangles(Mat rgbImage, RectangleShape areaOfInterest, ColorRange colorRange, double minArea) {
        return findShapes(rgbImage, areaOfInterest, colorRange, minArea, FitShape.RECTANGLE);
    }

    Mat pyr = new Mat();
    Mat hsv = new Mat();
    Mat thresholded = new Mat();
    Mat temp = new Mat();
    private List<Result> findShapes(Mat rgbImage, RectangleShape areaOfInterest, ColorRange colorRange, double minArea, FitShape shapeType) {

        // Do we want to process only a portion of the image?
        boolean imageSubset = (areaOfInterest != null);
        int fromRow = 0;
        int toRow = 0;
        int fromCol = 0;
        int toCol = 0;

        Mat rgb = null;
        if(imageSubset) {
            // If we want to process only a portion of the image, get the size of the subset
            fromRow = (int) areaOfInterest.getMinY();
            toRow = (int) areaOfInterest.getMaxY();
            fromCol = (int) areaOfInterest.getMinX();
            toCol = (int) areaOfInterest.getMaxX();

            // make sure it is within the size of the original image.
            fromRow = (fromRow < 0 ? 0 : fromRow);
            fromCol = (fromCol < 0 ? 0 : fromCol);
            toRow = (toRow > rgbImage.rows() ? rgbImage.rows() : toRow);
            toCol = (toCol > rgbImage.cols() ? rgbImage.cols() : toCol);

            // then extract the subset from the original image
            rgb = rgbImage.submat(fromRow, toRow, fromCol, toCol);
        } else {
            // Otherwise just use the original image (no subset)
            rgb = rgbImage;
        }

        // Convert the image from RGB to HSV. HSV is more consistent under different lighting conditions
       Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV, 3);

        // Blur the image to remove some initial noise
        Imgproc.blur(hsv, hsv, new Size(3, 3));

        // Now create a binary image (black and white, no grey). The part of the image that matches
        // the target color will be white, everything else will be black
        Core.inRange(hsv, new Scalar(colorRange.getLower()), new Scalar(colorRange.getUpper()), thresholded);

        // Now dilate the white portions to squeeze out any small black portions and to connect slightly
        // disconnected white areas.
        Imgproc.dilate(thresholded, temp, new Mat(), new Point(-1, -1), 3);

        // Now contract the white areas back down - this won't disconnect any white areas that were
        // connected during the dilate.
       Imgproc.erode(temp, thresholded, new Mat(), new Point(-1, -1), 3);

       // OpenCV has a function for finding the outside contours of white areas in binary images like
       // the one we just created. The contours are stored as arrays of (x,y) points that describe the
       // small lines creating the contour.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Result> results = new ArrayList<>();
        int negArea = (int) (minArea * -1);
        for(MatOfPoint mop : contours) {
            if(Imgproc.contourArea(mop, true) < negArea) {
                List<Point> points = mop.toList();

                if(imageSubset) {
                    for(Point p : points) {
                        p.x = (p.x + fromCol);
                        p.y = (p.y + fromRow);
                    }
                }
                MatOfPoint mop2 = new MatOfPoint();
                mop2.fromList(points);

                MatOfPoint2f mop2f = new MatOfPoint2f();
                mop2f.fromList(points);

                Result res = new Result();
                switch(shapeType) {
                    case CIRCLE:
                        Point center = new Point();
                        float[] rad = new float[1];

                        Imgproc.minEnclosingCircle(mop2f, center, rad);
                        res.setCircle(new CircleShape(center, rad[0]));
                        res.setShape(FitShape.CIRCLE);
                        break;
                    case RECTANGLE:
                        RotatedRect rr = Imgproc.minAreaRect(mop2f);
                        res.setRectangle(rr);
                        res.setShape(FitShape.RECTANGLE);
                        break;
                }

                res.setContour(mop2);
                res.setFrameSize(new RectangleShape().setDims(0, 0, rgbImage.cols(), rgbImage.rows()));
                results.add(res);
            }
        }

        return results;
    }

}