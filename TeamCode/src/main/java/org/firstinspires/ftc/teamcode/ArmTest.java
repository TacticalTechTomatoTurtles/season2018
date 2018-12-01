package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmTest extends LinearOpMode {
    // This one represents the main arm motor
    private DcMotor armMotorF;
    private DcMotor rackMotorF;


    @Override
    public void runOpMode() {

        // this is where were kinda connecting the dots between the variable and the real life motors
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        rackMotorF = hardwareMap.get(DcMotor.class, "motorX");

        // Send a message to the drivers phone that the variables are all set.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // program waits until the start button is pressed on the phone.
        waitForStart();

        //these variables hold the power values from the joystick
        double tgtPowerArm = 0;
        double tgtPowerRack = 0;

        //stops movement of robot quickly.
        armMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This is the main loop that runs tele-op and runs multiple times a second.
        //This keeps looping until the stop button is pressed.
        Timer t = new Timer();
        t.setCompareTime(1000);
        t.start();
        while (opModeIsActive()) {
            if(t.timeChecker()) {
                break;
            }
            // you are asking the game pad what your current position is of a certain joystick or button is
            tgtPowerArm = -0.0;
            tgtPowerRack = -1.0;

            // you are telling the robot to use those variables to set that power to the motors
            armMotorF.setPower(tgtPowerArm/2.5);
            rackMotorF.setPower(tgtPowerRack/2.5);

            // its sending the power of the motors to the phone
            telemetry.addData("Target Power Front Arm", tgtPowerArm);
            telemetry.addData("Target Power Rack Front", tgtPowerRack);

            // sending motors to the phone
            telemetry.addData("Motor Power Arm Front", armMotorF.getPower());
            telemetry.addData("Motor Power Rack Front", rackMotorF.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

        armMotorF.setPower(0);
        rackMotorF.setPower(0);
    }
}
