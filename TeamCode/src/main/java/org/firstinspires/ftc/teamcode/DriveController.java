package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveController {

    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;

    public void goForward(double power) {
        leftMotorB.setPower(power);
        leftMotorF.setPower(power);
        rightMotorB.setPower(power);
        rightMotorF.setPower(power);
    }
    public void stop() {
        leftMotorB.setPower(0);
        leftMotorF.setPower(0);
        rightMotorB.setPower(0);
        rightMotorF.setPower(0);
    }
    public void turn(int degrees) {
        leftMotorB.setPower(1);
        leftMotorF.setPower(1);
        rightMotorB.setPower(-1);
        rightMotorF.setPower(-1);
    }
}
