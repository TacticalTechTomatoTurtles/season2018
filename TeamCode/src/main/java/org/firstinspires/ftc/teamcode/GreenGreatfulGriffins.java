package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.vision.MineralIdentifier;
import org.firstinspires.ftc.teamcode.vision.Result;

import java.util.List;

@Autonomous
public class GreenGreatfulGriffins extends LinearOpMode {

    @Override
    public void runOpMode() {
//                                                                                                                                      ChickFilA < Culvers
        // creating new representobot object from the representobot class
        Representobot bot = new Representobot(this);

        // creating new mineral identifier to find the gold block
        MineralIdentifier minid = new MineralIdentifier();
        minid.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        // start the vision system
        minid.enable();

        // wait for the start button to be pushed
        waitForStart();

        // lowers the robot to the ground, and then starts calibrrating the gyro, and then
        // retracts the arm!
        bot.lowerFromLander();
        bot.startGyro();
        bot.retractLanderArm();

        // ASSERT! we are all detached from the lander

        // ask the vision sensors if it see any blocks
        // average the center of the block for 10 frames
        boolean blockFound = false;
        double centerx = 0;
        for(int i = 0; i < 10; i++) {
            List<Result> results = minid.getResults();
            if (results != null && results.size() == 1) {
                Result item = results.get(0);
                blockFound = true;
                centerx = centerx + item.getCenter().x;
            }
        }
        centerx = centerx / 10;
        log("x value " + centerx);

        // turn off the vision sensor
        minid.disable();

        double width = 480;
        double half = width / 2;
        double excludeLeft = 120;

        // see if we got a hit from the vision system
        if (blockFound && centerx < half && centerx > excludeLeft) {
            // block is in front = go forward 18 inches
            bot.goForward(0.3, 25);
        } else if (blockFound && centerx > half) {
            //turn right
            bot.turnRight(40, 0.2);
            bot.goForward(0.3, 14);
            bot.turnLeft(80, 0.2);
            bot.goForward(0.3, 25);
        } else {
            // go left
            bot.turnLeft(40, 0.2);
            bot.goForward(0.3, 14);
            bot.turnRight(70, 0.2);
            bot.goForward(0.3, 25);
        }

    }

    private void log(String message) {
        telemetry.addData("log", message);
        telemetry.update();
    }
}

