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

    private boolean updated = false;
    private List<Result> procResults = null;

    public synchronized boolean setUpdated(boolean val) {
        boolean old = updated;
        updated = val;

        return old;
    }

    public synchronized List<Result> getResults() {
        return procResults;
    }

    private synchronized void setResults(List<Result> res) {
        this.procResults = res;
    }

    ImageProcessor improc = new ImageProcessor();
    // This is called every camera frame.

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

            ColorRange colorRange = new ColorRange();
        //colorRange.setLower(98.03, 253.51, 94.87);
        //colorRange.setUpper(107.47, 256.19, 257.43);

        //colorRange.setLower(6.09, 207.04, 121.87);
        //colorRange.setUpper(21.13, 261.72, 222.71);
        colorRange.setLower(2.33, 193.37, 96.66);
        colorRange.setUpper(24.89, 275.39, 247.92);

        List<Result> results = improc.findRectangles(rgba, null, colorRange, 200);
        setResults(results);

        List<MatOfPoint> contours = new ArrayList<>();
        for(Result r : results) {
            contours.add(r.getContour());
        }

        Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 255, 0), 2, 8);

        for(Result res : results) {
            CircleShape c = res.getCircle();
            if(c != null) {
                Imgproc.circle(rgba, c.getCenter(), (int) c.getRadius(), new Scalar(255, 0, 0));
                Imgproc.circle(rgba, c.getCenter(), 3, new Scalar(255, 0, 0));
            }

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

        return rgba; // display the image seen by the camera
    }
}
