package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AndyIsMakingUsDoManualLabor extends LinearOpMode {
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double tgtPowerLB = 0;
        double tgtPowerRB = 0;
        double tgtPowerLF = 0;
        double tgtPowerRF = 0;
        double tgtPowerArm = 0;
        double tgtPowerSteve = 0;
       // double tgtPowerNegSteve = 0;

        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (opModeIsActive()) {
            tgtPowerLB = gamepad1.left_stick_y;
            tgtPowerRF = -gamepad1.right_stick_y;
            tgtPowerLF = gamepad1.left_stick_y;
            tgtPowerRB = -gamepad1.right_stick_y;
            tgtPowerArm = -gamepad2.right_stick_y;
            tgtPowerSteve = this.gamepad2.left_trigger;

            leftMotorF.setPower(tgtPowerLF);
            leftMotorB.setPower(tgtPowerLB);
            rightMotorF.setPower(tgtPowerRF);
            rightMotorB.setPower(tgtPowerRB);
            armMotorF.setPower(tgtPowerArm);
            rightServoF.setPosition(.75);

            if(gamepad1.y){
                changePower(.10);
            }
            if(gamepad1.a) {
                changePower(-.10);
            }

            telemetry.addData("Target Power Left Back", tgtPowerLB);
            telemetry.addData("Target Power Right Back", tgtPowerRB);
            telemetry.addData("Target Power Left Front", tgtPowerLF);
            telemetry.addData("Target Power Right Front", tgtPowerRF);
            telemetry.addData("Target Power Front Arm", tgtPowerArm);
            telemetry.addData("Servo Right Front", tgtPowerSteve);
           // telemetry.addData("Servo Negative Right Front", tgtPowerNegSteve);

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

