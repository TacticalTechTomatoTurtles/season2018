package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class AndyIsMakingUsDoManualLabor extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;


    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        rightMotor = hardwareMap.get(DcMotor.class, "motor2");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double tgtPowerL = 0;
        double tgtPowerR = 0;
        while (opModeIsActive()) {
            tgtPowerL = this.gamepad1.left_stick_y;
            tgtPowerR = -this.gamepad1.right_stick_y;
            leftMotor.setPower(tgtPowerL);
            rightMotor.setPower(tgtPowerR);
            telemetry.addData("Target Power", tgtPowerL);
            telemetry.addData("Target Power", tgtPowerR);
            telemetry.addData("Motor Power", leftMotor.getPower());
            telemetry.addData("Motor Power", rightMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

}