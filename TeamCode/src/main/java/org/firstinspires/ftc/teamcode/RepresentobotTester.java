package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RepresentobotTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        Representobot bot = new Representobot(this);


        // wait for the start button to be pushed
        waitForStart();

        bot.startGyro();

        //bot.goForward(0.4,10);
        //bot.turnRight(40, 0.2);
        bot.forwardToWall(6, 0.2);
    }
}
