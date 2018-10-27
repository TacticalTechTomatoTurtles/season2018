package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class BlueBase extends LinearOpMode {

    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;
    private DcMotor armMotorF;
    private Servo rightServoF;

    @Override
    public void runOpMode() {
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        rightServoF = hardwareMap.get(Servo.class,"steve");


        waitForStart();
        double tgtPowerLB = 0;
        double tgtPowerRB = 0;
        double tgtPowerLF = 0;
        double tgtPowerRF = 0;
        double tgtPowerArm = 0;

        while (opModeIsActive()) {
            leftMotorF.setPower(tgtPowerLF);
            leftMotorB.setPower(tgtPowerLB);
            rightMotorF.setPower(tgtPowerRF);
            rightMotorB.setPower(tgtPowerRB);
            armMotorF.setPower(tgtPowerArm);
            rightServoF.setPosition(.75);

            // go forward x inches

            // turn right 90 degrees

            // go forward y inches

            // drop the figure

            //back into the crater
        }
    }
}
