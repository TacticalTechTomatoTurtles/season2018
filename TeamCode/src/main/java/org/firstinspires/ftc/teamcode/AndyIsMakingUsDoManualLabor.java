package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AndyIsMakingUsDoManualLabor extends LinearOpMode {

    // represents the 4 wheel motors
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;

    // This one represents the main arm motor
    private DcMotor armMotorF;
    private Servo rightServoF;
    private Servo iconServoF;
    private Servo armServo;
    private DcMotor rackMotorF;


    @Override
    public void runOpMode() {

        // this is where were kinda connecting the dots between the variable and the real life motors
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        rackMotorF = hardwareMap.get(DcMotor.class, "motorX");
        rightServoF = hardwareMap.get(Servo.class,"steve");
        iconServoF = hardwareMap.get(Servo.class,"iconDropServo");
        armServo = hardwareMap.get(Servo.class,"armServo");


        // Send a message to the drivers phone that the variables are all set.
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
        double tgtPowerRack = 0;
        double tgtPowerSteve = 0;
        double tgtPowerIcon = 0;
        double servoPowerKeeper = 0;
        double tgtPowerArmServo = 0;
        double factor = 2;
       // double tgtPowerNegSteve = 0;

        //stops movement of robot quickly.
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //This is the main loop that runs tele-op and runs multiple times a second.
        //This keeps looping until the stop button is pressed.
        rightServoF.setPosition(0);
        while (opModeIsActive()) {
            // you are asking the game pad what your current position is of a certain joystick or button is
            tgtPowerLB = gamepad1.left_stick_y;
            tgtPowerRF = -gamepad1.right_stick_y;
            tgtPowerLF = gamepad1.left_stick_y;
            tgtPowerRB = -gamepad1.right_stick_y;
            tgtPowerArm = -gamepad2.right_stick_y;
            tgtPowerRack = -gamepad2.left_stick_y;
            tgtPowerSteve = this.gamepad2.left_trigger;
            tgtPowerIcon = this.gamepad1.left_trigger;
            tgtPowerArmServo = servoPowerKeeper;

            if(this.gamepad2.dpad_up){
                servoPowerKeeper = servoPowerKeeper + 0.01;
                if(servoPowerKeeper > 1) {
                    servoPowerKeeper = 1;
                }
            }else if(this.gamepad2.dpad_down) {
                servoPowerKeeper = servoPowerKeeper - 0.01;
                if(servoPowerKeeper<0){
                    servoPowerKeeper = 0;
                }
            }


            // determine the denominator based on the button

            if(gamepad1.y){
                factor = 1;
            }else if(gamepad1.a) {
                factor = 3;
            }else if(gamepad1.x){
                factor = 2;
            }

            // you are telling the robot to use those variables to set that power to the motors
            leftMotorF.setPower(tgtPowerLF/factor);
            leftMotorB.setPower(tgtPowerLB/factor);
            rightMotorF.setPower(tgtPowerRF/factor);
            rightMotorB.setPower(tgtPowerRB/factor);
            armMotorF.setPower(tgtPowerArm/2.5);
            rackMotorF.setPower(tgtPowerRack);
            rightServoF.setPosition(tgtPowerSteve);
            iconServoF.setPosition(tgtPowerIcon);
            armServo.setPosition(tgtPowerArmServo);


            // its sending the power of the motors to the phone
            telemetry.addData("Target Power Left Back", tgtPowerLB);
            telemetry.addData("Target Power Right Back", tgtPowerRB);
            telemetry.addData("Target Power Left Front", tgtPowerLF);
            telemetry.addData("Target Power Right Front", tgtPowerRF);
            telemetry.addData("Target Power Front Arm", tgtPowerArm);
            telemetry.addData("Target Power Rack Front", tgtPowerRack);
            telemetry.addData("Servo Right Front", tgtPowerSteve);
            telemetry.addData("Arm Servo", tgtPowerArmServo);
            telemetry.addData("Icon Drop Servo", tgtPowerIcon);

            //telemetry.addData("Servo Negative Right Front", tgtPowerNegSteve);

            // sending motors to the phone
            telemetry.addData("Motor Power Left Back", leftMotorB.getPower());
            telemetry.addData("Motor Power Left Front", leftMotorF.getPower());
            telemetry.addData("Motor Power Right Back", rightMotorB.getPower());
            telemetry.addData("Motor Power Right Front", rightMotorF.getPower());
            telemetry.addData("Motor Power Arm Front", armMotorF.getPower());
            telemetry.addData("Motor Power Rack Front", rackMotorF.getPower());
            telemetry.addData("Servo Position", rightServoF.getPosition());
            telemetry.addData("Servo Position", iconServoF.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        armMotorF.setPower(0);
        rackMotorF.setPower(0);
        rightServoF.setPosition(0);
    }

    public void changePower(double nPower){
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

