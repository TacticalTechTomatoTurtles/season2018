package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
public class GoToCrater extends LinearOpMode {
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

        Timer myTimer = new Timer();
        long mili = inchesToTime(40.0);
        myTimer.setCompareTime(mili);
        myTimer.start();
        while (opModeIsActive()) {
            if (myTimer.timeChecker()) {
                leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            } else {
                leftMotorB.setPower(.5);
                leftMotorF.setPower(.5);
                rightMotorB.setPower(-.5);
                rightMotorF.setPower(-.5);
            }
        }
    }
    public long inchesToTime(double inches) {
       return (long) (0.0384 * inches * 1000.0);
    }
}



