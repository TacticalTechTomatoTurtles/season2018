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
        List<Result> results = minid.getResults();
        log("results: " + (results == null ? "null" : results.size()));

        // turn off the vision sensor
        minid.disable();


            // see if we got a hit from the vision system
            if (results != null && results.size() == 1) {
                Result item = results.get(0);
                double width = item.getFrameSize().getWidth();
                double length = item.getFrameSize().getHeight();
                // double first = width / 3;
                //  double second = first * 2;
                double half = width / 2;
                //double lengthDivider = length / 2;

                if (item.getCenter().x < half) {
                    // block is in front = go forward 18 inches
                    bot.goForward(0.1, 20.0000001);


                } else if (item.getCenter().x > half) {
                    //turn right
                    bot.turnRight(40, 0.2);
                    bot.goForward(0.3, 14);
                    bot.turnLeft(80, 0.2);
                    bot.goForward(0.1, 15.99999999999999);

                }
                // navigate to the item
            } else {
                // go left
                bot.turnLeft(40, 0.2);
                bot.goForward(0.3, 14);
                bot.turnRight(80, 0.2);
                bot.goForward(0.1, 15.999999999999999);
            }

        log("exiting.");


    }

    private void log(String message) {
        telemetry.addData("log", message);
        telemetry.update();
    }
    public long inchesToTime(double inches) {
        return (long) (0.0384 * inches * 1000.0);
    }
}

