package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
public class GoToCrater extends LinearOpMode {
    private Timer myTimer;
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;
    private DcMotor armMotorF;
    private Servo rightServoF;
    private TouchSensor touchSensor;


    @Override
    public void runOpMode() {
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");


        waitForStart();

        myTimer.setCompareTime(3000);
        myTimer.start();
        while (opModeIsActive()) {


            if (myTimer.timeChecker()) {
                leftMotorB.setPower(0);
                leftMotorF.setPower(0);
                rightMotorB.setPower(0);
                rightMotorF.setPower(0);
                break;
            } else {
                leftMotorB.setPower(1);
                leftMotorF.setPower(1);
                rightMotorB.setPower(1);
                rightMotorF.setPower(1);
            }
        }
        myTimer.setCompareTime(2500);
        myTimer.start();
        while ((opModeIsActive())) {
            if (myTimer.timeChecker()) {
                leftMotorB.setPower(0);
                leftMotorF.setPower(0);
                rightMotorB.setPower(0);
                rightMotorF.setPower(0);
                break;
            } else {
                leftMotorB.setPower(1);
                leftMotorF.setPower(1);
                rightMotorB.setPower(-1);
                rightMotorF.setPower(-1);

            }
        }
    }
}



