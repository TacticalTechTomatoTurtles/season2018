package org.firstinspires.ftc.teamcode.vision;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class HSVSampler extends OpenCVPipeline {

    private byte[] vals = {0, 0, 0};
    private Object valLock = new Object();
    private Mat hsv = new Mat();

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        int rows = rgba.rows();
        int cols = rgba.cols();

        Point mid = new Point(rows / 2.0, cols / 2.0);
        Imgproc.circle(rgba, mid, 3, new Scalar(0, 255, 0));

        byte[] v = new byte[3];
        hsv.get((int) mid.y, (int) mid.x, v);
        setValues(v);

        return rgba; // display the image seen by the camera
    }

    private void setValues(byte[] v) {
        synchronized (valLock) {
            System.arraycopy(v, 0, vals, 0, 3);
        }
    }

    public byte[] getValues() {
        byte[] ret = new byte[3];
        synchronized (valLock) {
            System.arraycopy(vals, 0, ret, 0, 3);
        }
        return ret;
    }
}
