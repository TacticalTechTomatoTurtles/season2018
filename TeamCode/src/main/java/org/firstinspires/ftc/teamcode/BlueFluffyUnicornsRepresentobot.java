package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.vision.MineralIdentifier;
import org.firstinspires.ftc.teamcode.vision.Result;

import java.util.List;

@Autonomous
public class BlueFluffyUnicornsRepresentobot extends LinearOpMode {

    @Override
    public void runOpMode() {
        Representobot bot = new Representobot(this);

        MineralIdentifier minid = new MineralIdentifier();
        minid.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        // start the vision system
        minid.enable();

        // wait for the start button to be pushed
        waitForStart();

        bot.lowerFromLander();
        bot.startGyro();
        bot.retractLanderArm();

        // ASSERT! we are all detached from the lander

        List<Result> results = minid.getResults();
        log("results: " + (results == null ? "null" : results.size()));
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
                    // CHECK: block is in front = go forward 18 inches
                    bot.goForward(0.3, 9);

                    // CHECK: go forward until 5 inches from wall
                    bot.forwardToWall(0.3, 8);

                } else if (item.getCenter().x > half) {
                    //turn right
                    bot.turnRight(40, 0.3);
                    bot.goForward(0.3, 14);
                    bot.turnLeft(80, 0.3);
                    bot.forwardToWall(10, 0.3);
                    //bot.goForward(0.3, 16);
                    //bot.turnLeft(35, 0.3);
                    //bot.goForward(0.3, 5);
                    //bot.turnLeft(17, 0.3);
                    //bot.goForward(0.3, 5);

                }
                // navigate to the item
            } else {
                // go left
                bot.turnLeft(41, 0.2);
                bot.goForward(0.5, 9);
                bot.turnRight(35, 0.2);
                bot.goForward(0.25, 5);
                bot.turnLeft(17, 0.2);
            }



        log("exiting.");
        // drops the icon, if it doesn't work shakes off icon
        bot.dropIcon();

    }

    private void log(String message) {
        telemetry.addData("log", message);
        telemetry.update();
    }
    public long inchesToTime(double inches) {
        return (long) (0.0384 * inches * 1000.0);
    }
}
