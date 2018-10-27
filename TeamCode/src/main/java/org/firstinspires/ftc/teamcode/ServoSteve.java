package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoSteve extends LinearOpMode {
    private Servo rightServoF;

    @Override
    public void runOpMode() {
        rightServoF = hardwareMap.get(Servo.class, "steve");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double tgtPowerSteve = 0;

        while (opModeIsActive()) {
            tgtPowerSteve = gamepad1.left_trigger;
            rightServoF.setPosition(tgtPowerSteve);
            rightServoF.setPosition(0.75);


            telemetry.addData("Servo Position", rightServoF.getPosition());
            telemetry.addData("Target Power", tgtPowerSteve);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }


    }

