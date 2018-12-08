package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
public class SimpleDetach extends LinearOpMode {
    private Timer myTimer;
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;
    private DcMotor armMotorF;
    private DcMotor rackMotorF;
    private Servo rightServoF;
    private TouchSensor Tanner;

    @Override
    public void runOpMode() {

        // get all the hardware
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        rackMotorF = hardwareMap.get(DcMotor.class, "motorX");
        rightServoF = hardwareMap.get(Servo.class, "steve");
        Tanner = hardwareMap.get(TouchSensor.class, "touchSensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        Gyro gyro = new Gyro(imu, this);
        myTimer = new Timer();

        //stops movement of robot quickly.
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // wait for the start button to be pushed
        waitForStart();

        // raise the arm so the robot touches the ground
        myTimer.setCompareTime(2300);
        rackMotorF.setPower(-1);
        myTimer.start();
        while (opModeIsActive()) {
           if(myTimer.timeChecker()){
               break;
            }
        }
        rackMotorF.setPower(0);

        // now that the robot is on the ground calibrate the gyro
        gyro.StartGyro();


        // -- turn left to get off the lander latch --

        // tell the gyro we are turning left and reset the measured angle to 0
        gyro.resetWithDirection(Gyro.LEFT);

        // start the motors turning left
        leftMotorF.setPower(0.2);
        leftMotorB.setPower(0.2);
        rightMotorF.setPower(0.2);
        rightMotorB.setPower(0.2);

        // loop until the robot turns 25 degrees
        while (opModeIsActive()) {
            if(gyro.getAngle() >= 25){
                break;
            }
        }

        // turn off the motors after the turn
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

        // -- lower the rack arm --
        myTimer.setCompareTime(2300);
        rackMotorF.setPower(1);
        myTimer.start();
        while (opModeIsActive()) {
            if(myTimer.timeChecker()){
                break;
            }
        }
        rackMotorF.setPower(0);

        // -- turn right to line back up with the crater --

        // tell the gyro we are turning right and reset the measured angle to 0
        gyro.resetWithDirection(Gyro.RIGHT);

        // start the motors turning right
        leftMotorF.setPower(-0.2);
        leftMotorB.setPower(-0.2);
        rightMotorF.setPower(-0.2);
        rightMotorB.setPower(-0.2);

        // loop until the robot turns 25 degrees
        while (opModeIsActive()) {
            if(gyro.getAngle() <= -25){
                break;
            }
        }


        // -- go forward a few inches to get clear of the lander --
        myTimer.setCompareTime(inchesToTime(40));
        leftMotorF.setPower(-0.25);
        leftMotorB.setPower(-0.25);
        rightMotorF.setPower(0.25);
        rightMotorB.setPower(0.25);
        myTimer.start();
        while (opModeIsActive()) {
                if (myTimer.timeChecker()){
                    break;
            }
        }


        // -- back into the crater --
        myTimer.setCompareTime(inchesToTime(100));
        leftMotorF.setPower(-0.25);
        leftMotorB.setPower(-0.25);
        rightMotorF.setPower(0.25);
        rightMotorB.setPower(0.25);

        myTimer.start();
        while (opModeIsActive()) {
            if (myTimer.timeChecker()){
                break;
            }
        }

        // hopefully we are in the crater now!
        // turn off the motors
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }

    public long inchesToTime(double inches) {
        return (long) (0.0384 * inches * 1000.0);
    }
}
