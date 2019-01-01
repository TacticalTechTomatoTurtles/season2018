package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.vision.HSVSampler;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp
public class ColorSampler extends LinearOpMode {

   private static final int SAMPLE_TIME = 10;

    @Override
    public void runOpMode() {
        HSVSampler sampler = new HSVSampler();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        sampler.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        // start the vision system
        sampler.enable();
        waitForStart();

        Timer t = new Timer();
        t.setCompareTime(SAMPLE_TIME * 1000);
        t.start();

        List<Integer> v0 = new ArrayList<>();
        List<Integer> v1 = new ArrayList<>();
        List<Integer> v2 = new ArrayList<>();
        while(!t.timeChecker() && opModeIsActive()) {

            byte[] vals = sampler.getValues();

            v0.add(vals[0] & 0xFF);
            v1.add(vals[1] & 0xFF);
            v2.add(vals[2] & 0xFF);
        }

        sampler.disable();

        double[] stats = new double[2];
        getStats(v0, stats);
        telemetry.addData("V0", String.format(Locale.getDefault(), "(%1$,.2f, %2$,.2f)", stats[0], stats[1]));
        getStats(v1, stats);
        telemetry.addData("V1", String.format(Locale.getDefault(), "(%1$,.2f, %2$,.2f)", stats[0], stats[1]));
        getStats(v2, stats);
        telemetry.addData("V2", String.format(Locale.getDefault(), "(%1$,.2f, %2$,.2f)", stats[0], stats[1]));
        telemetry.update();

        while(opModeIsActive()) {
            idle();
        }
    }

    private double[] getStats(List<Integer> v0, double[] stats) {
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

        stats[0] = avg;
        stats[1] = std;

        return stats;
    }

}
