package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.vision.ColorRange;
import org.firstinspires.ftc.teamcode.vision.MineralIdentifier;
import org.firstinspires.ftc.teamcode.vision.Result;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp
public class VisionTest extends LinearOpMode {
    private ExampleBlueVision blueVision;



    public void runOpMode() {

        MineralIdentifier minid = new MineralIdentifier();

        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        minid.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        // start the vision system
        minid.enable();
        log("waiting for start");
        waitForStart();

        Timer t = new Timer();
        t.setCompareTime(5000);
        t.start();
        log("timer started");

        //while(opModeIsActive()) {
        while(opModeIsActive()) {
            List<Result> results = minid.getResults();
            log("results: " + (results == null ? "null" : results.size()));
        }

        minid.disable();
        log("exiting.");
    }

    private void log(String message) {
        telemetry.addData("log", message);
        telemetry.update();
    }

    public void runOpMode2() {
        blueVision = new ExampleBlueVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowCountours(false);

        // start the vision system
        blueVision.enable();
        log("waiting for start");
        waitForStart();

        Timer t = new Timer();
        t.setCompareTime(10000);
        t.start();
        log("timer started");

        //while(opModeIsActive()) {
        List<Integer> v0 = new ArrayList<>();
        List<Integer> v1 = new ArrayList<>();
        List<Integer> v2 = new ArrayList<>();
        while(!t.timeChecker()) {
            // update the settings of the vision pipeline
            //log("1");
            //blueVision.setShowCountours(gamepad1.x);
            //blueVision.setShowCountours(true);
            // get a list of contours from the vision system
            //log("2");
            //List<MatOfPoint> contours = blueVision.getContours();
            //log("3");
            //for (int i = 0; i < contours.size(); i++) {
                // get the bounding rectangle of a single contour, we use it to get the x/y center
                // yes there's a mass center using Imgproc.moments but w/e
             //   log("4");
             //   Rect boundingRect = Imgproc.boundingRect(contours.get(i));
             //   log("5");
             //   telemetry.addData("contour" + Integer.toString(i),
             //           String.format(Locale.getDefault(), "(%d, %d)", (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));
             //   log("6");

            byte[] vals = blueVision.getValues();
            log("[" + (vals[0] & 0xFF) + ", " + (vals[1] & 0xFF) + ", " + (vals[2] & 0xFF) + "]");
            v0.add(vals[0] & 0xFF);
            v1.add(vals[1] & 0xFF);
            v2.add(vals[2] & 0xFF);

            //log("7");
        }

        blueVision.disable();

        double[] stats;
        stats = getStats(v0);
        telemetry.addData("V0", String.format(Locale.getDefault(), "(%1$,.2f, %2$,.2f)", stats[0], stats[1]));
        stats = getStats(v1);
        telemetry.addData("V1", String.format(Locale.getDefault(), "(%1$,.2f, %2$,.2f)", stats[0], stats[1]));
        stats = getStats(v2);
        telemetry.addData("V2", String.format(Locale.getDefault(), "(%1$,.2f, %2$,.2f)", stats[0], stats[1]));
        telemetry.update();

        while(opModeIsActive()) {

        }
    }

    private double[] getStats(List<Integer> v0) {
        double avg = 0;
        double std = 0;
        for(int i = 0; i < v0.size(); i++) {
            avg += v0.get(i);
        }
        avg /= v0.size();

        for(int i = 0; i < v0.size(); i++) {
            std += Math.pow(v0.get(i) - avg, 2);
        }
        std = Math.sqrt(std / v0.size());

        double[] results = new double[2];
        results[0] = avg;
        results[1] = std;

        return results;
    }
}
