package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Sensor Test", group = "Sensor")
public class TestTouch extends LinearOpMode {

    TouchSensor digitalTouch;

    @Override
    public void runOpMode() {


        digitalTouch = hardwareMap.get(TouchSensor.class, "touchSensor");

        digitalTouch.isPressed();

        waitForStart();

        while (opModeIsActive()) {

            if (digitalTouch.isPressed() == true) {
                telemetry.addData("Touch Sensor:", " Is Pressed");
            } else {
                telemetry.addData("Touch Sensor:", " Is Not Pressed");
            }

            telemetry.update();
        }
    }
}