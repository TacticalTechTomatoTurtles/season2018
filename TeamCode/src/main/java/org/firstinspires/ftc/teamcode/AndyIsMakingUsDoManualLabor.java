package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AndyIsMakingUsDoManualLabor extends LinearOpMode {

    // represents the 4 wheel motorss
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;

    // This one represents the main arm motor
    private DcMotor armMotorF;
    private Servo rightServoF;


    @Override
    public void runOpMode() {

        // this is where were kinda connecting the dots between the variable and the real life motors
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        rightServoF = hardwareMap.get(Servo.class,"steve");

        // Send a messgae to the drivers phone that the variables are all set.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // program waits until the start button is pressed on the phone.
        waitForStart();

        //these variables hold the power values from the joystick
        double tgtPowerLB = 0;
        double tgtPowerRB = 0;
        double tgtPowerLF = 0;
        double tgtPowerRF = 0;
        double tgtPowerArm = 0;
        double tgtPowerSteve = 0;
       // double tgtPowerNegSteve = 0;

        //stops movement of robot quickly.
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //This is the main loop that runs teleop and runs multiple times a second.
        //This keeps looping until the stop button is pressed.
        rightServoF.setPosition(0);
        while (opModeIsActive()) {
            // you are asking the game pad what your current position is of a certain joystick or botton is
            tgtPowerLB = gamepad1.left_stick_y;
            tgtPowerRF = -gamepad1.right_stick_y;
            tgtPowerLF = gamepad1.left_stick_y;
            tgtPowerRB = -gamepad1.right_stick_y;
            tgtPowerArm = -gamepad2.right_stick_y;
            tgtPowerSteve = this.gamepad2.left_trigger;


            // you are telling the robot to use those variables to set that power to the motors
            leftMotorF.setPower(tgtPowerLF/2);
            leftMotorB.setPower(tgtPowerLB/2);
            rightMotorF.setPower(tgtPowerRF/2);
            rightMotorB.setPower(tgtPowerRB/2);
            armMotorF.setPower(tgtPowerArm/2.5);
            rightServoF.setPosition(tgtPowerSteve);

//            if(gamepad1.y){
//                changePower(.10);
//          }
  //          if(gamepad1.a) {
  //          changePower(-.10);
//            }

            // its sending the power of the motors to the phone
            telemetry.addData("Target Power Left Back", tgtPowerLB);
            telemetry.addData("Target Power Right Back", tgtPowerRB);
            telemetry.addData("Target Power Left Front", tgtPowerLF);
            telemetry.addData("Target Power Right Front", tgtPowerRF);
            telemetry.addData("Target Power Front Arm", tgtPowerArm);
            telemetry.addData("Servo Right Front", tgtPowerSteve);
           // telemetry.addData("Servo Negative Right Front", tgtPowerNegSteve);

            // sending motors to the phone
            telemetry.addData("Motor Power Left Back", leftMotorB.getPower());
            telemetry.addData("Motor Power Left Front", leftMotorF.getPower());
            telemetry.addData("Motor Power Right Back", rightMotorB.getPower());
            telemetry.addData("Motor Power Right Front", rightMotorF.getPower());
            telemetry.addData("Motor Power Arm Front", armMotorF.getPower());
            telemetry.addData("Servo Position", rightServoF.getPosition());
            telemetry.addData("Target Power", tgtPowerSteve);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        armMotorF.setPower(0);
        rightServoF.setPosition(0);
    }

    public void changePower(Double nPower){
        leftMotorB.setPower((1+nPower)*leftMotorB.getPower());
        telemetry.addData("Left back motor", leftMotorB.getPower());
        leftMotorF.setPower((1+nPower)*leftMotorF.getPower());
        telemetry.addData("Left front motor", leftMotorF.getPower());
        rightMotorB.setPower((1+nPower)*rightMotorB.getPower());
        telemetry.addData("Right back motor", rightMotorB.getPower());
        rightMotorF.setPower((1+nPower)*rightMotorF.getPower());
        telemetry.addData("Right front motor is TURBO CHARGED", rightMotorF.getPower());
    }

}

