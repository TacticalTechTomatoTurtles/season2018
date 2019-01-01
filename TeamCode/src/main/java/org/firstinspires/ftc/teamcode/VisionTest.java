package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.vision.MineralIdentifier;
import org.firstinspires.ftc.teamcode.vision.Result;

import java.util.List;

@TeleOp
public class VisionTest extends LinearOpMode {

    public void runOpMode() {

        MineralIdentifier minid = new MineralIdentifier();

        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        minid.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        // start the vision system
        minid.enable();
        log("waiting for start");
        waitForStart();

        while(opModeIsActive()) {
            List<Result> results = minid.getResults();
            log("results: " + (results == null ? "null" : results.size()));

            if(results != null && results.size() == 1) {
                Result item = results.get(0);

                // navigate to the item
            } else {
                // look for the item
            }
        }

        minid.disable();
        log("exiting.");
    }

    private void log(String message) {
        telemetry.addData("log", message);
        telemetry.update();
    }

}
