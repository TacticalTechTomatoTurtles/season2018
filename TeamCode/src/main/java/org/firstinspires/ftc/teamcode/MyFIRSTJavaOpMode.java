package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private DcMotor motorTest;

    @Override
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotor.class, "motor1");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motorTest.setPower(1);
            telemetry.addData("Target Power", 1);
            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
        motorTest.setPower(0);
    }
}
