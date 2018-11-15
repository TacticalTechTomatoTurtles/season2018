package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveTest extends LinearOpMode {
    // represents the 4 wheel motorss
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;

    @Override
    public void runOpMode() {

        // this is where were kinda connecting the dots between the variable and the real life motors
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");

        // Send a messgae to the drivers phone that the variables are all set.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // program waits until the start button is pressed on the phone.
        waitForStart();

        //these variables hold the power values from the joystick
        //double tgtPowerLB = -1;
        //double tgtPowerRB = 1;
        //double tgtPowerLF = -1;
        //double tgtPowerRF = 1;

        double tgtPowerLB = 1;
        double tgtPowerRB = 1;
        double tgtPowerLF = 1;
        double tgtPowerRF = 1;

        //stops movement of robot quickly.
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Timer samfishy = new Timer();
        samfishy.setCompareTime(649);
        samfishy.start();
        //This is the main loop that runs teleop and runs multiple times a second.
        //This keeps looping until the stop button is pressed.
        while (opModeIsActive()) {

            // you are telling the robot to use those variables to set that power to the motors
            leftMotorF.setPower(tgtPowerLF / 2);
            leftMotorB.setPower(tgtPowerLB / 2);
            rightMotorF.setPower(tgtPowerRF / 2);
            rightMotorB.setPower(tgtPowerRB / 2);

            if(samfishy.timeChecker()) {
                break;
            }
        }
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }
}
