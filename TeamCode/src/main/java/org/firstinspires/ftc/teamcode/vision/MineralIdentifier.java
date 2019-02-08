package org.firstinspires.ftc.teamcode.vision;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MineralIdentifier extends OpenCVPipeline {

    private List<Result> procResults = null;
    private ImageProcessor improc = new ImageProcessor();
    private Tracker tracker = new Tracker();

    public synchronized List<Result> getResults() {
        return procResults;
    }

    private synchronized void setResults(List<Result> res) {
        this.procResults = res;
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        // Set the HSV color range we want to look for
        ColorRange colorRange = new ColorRange();
        colorRange.setLower(2.33, 193.37, 96.66);
        colorRange.setUpper(24.89, 275.39, 247.92);

        RectangleShape somethingMeaningful = new RectangleShape();
        somethingMeaningful.setDims(0, 380.0, 480.0, 640.0);

        // Find objects in the frame that are the right color
        // area of interest can be changed to only see the bottom half. This will fix the issue with seeing extra blocks.
        List<Result> results = improc.findRectangles(rgba, somethingMeaningful, colorRange, 200);
        tracker.update(results);
        Result tracked = tracker.getTrackedItem();

        results = new ArrayList<>();
        if(tracked != null) {
            results.add(tracked);
        }

        // Set the results so they can be later retrieved by the robot code.
        // This code is working in a different thread than the robot code
        setResults(results);

        // Get all of the object outlines from the results and draw them on the frame
        List<MatOfPoint> contours = new ArrayList<>();
        for(Result r : results) {
            contours.add(r.getContour());
        }
        Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 255, 0), 2, 8);

        // Draw the circles and/or rectangles that contain the outlines
        for(Result res : results) {

            // If there is a bounding circle for the outline, draw it and its center
            CircleShape c = res.getCircle();
            if(c != null) {
                Imgproc.circle(rgba, c.getCenter(), (int) c.getRadius(), new Scalar(255, 0, 0));
                Imgproc.circle(rgba, c.getCenter(), 3, new Scalar(255, 0, 0));
            }

            // If there is a bounding retangle for the outline, draw it and its center
            RotatedRect r = res.getRectangle();
            if(r != null) {
                Point[] verts = new Point[4];
                r.points(verts);
                MatOfPoint m = new MatOfPoint();
                m.fromArray(verts);

                Imgproc.drawContours(rgba, Arrays.asList(m), -1, new Scalar(255, 0, 0));
                Imgproc.circle(rgba, r.center, 3, new Scalar(255, 0, 0));
            }
        }

        // Now return the frame with the outlines and shapes drawn on it to the pipeline to be displayed on the phone
        return rgba;
    }
}
