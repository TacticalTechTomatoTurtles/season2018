package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.MineralIdentifier;

public class Representobot {
    private DistanceSensor sensorRange;
    private Timer myTimer;
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;
    private DcMotor armMotorF;
    private DcMotor rackMotorF;
    private Servo rightServoF;
    private Servo leftServoF;
    private TouchSensor Tanner;
    private Servo iconServo;
    private Gyro gyro;
    private LinearOpMode opMode;
    ModernRoboticsI2cRangeSensor rangeSensor;

    public Representobot(LinearOpMode om) {
        this.opMode = om;

        leftMotorB = opMode.hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = opMode.hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = opMode.hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = opMode.hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = opMode.hardwareMap.get(DcMotor.class, "motor4");
        rackMotorF = opMode.hardwareMap.get(DcMotor.class, "motorX");
        rightServoF = opMode.hardwareMap.get(Servo.class, "steve");
        leftServoF = opMode.hardwareMap.get(Servo.class, "iconDropServo");
        BNO055IMU imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro = new Gyro(imu, opMode);
        myTimer = new Timer();
        iconServo = opMode.hardwareMap.get(Servo.class, "alex");
        rangeSensor = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");

        //stops movement of robot quickly.
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void startGyro(){
         gyro.StartGyro();

     }

    public void lowerFromLander() {
        myTimer.setCompareTime(2300);
        rackMotorF.setPower(-1);
        myTimer.start();
        while (opMode.opModeIsActive()) {
            if (myTimer.timeChecker()) {
                break;
            }
        }
        rackMotorF.setPower(0);
    }

    public void retractLanderArm() {
        // -- turn left to get off the lander latch --
        turnLeft(25, 0.3);

        // -- lower the rack arm --
        myTimer.setCompareTime(2100);
        rackMotorF.setPower(1);
        myTimer.start();
        while (opMode.opModeIsActive()) {
            if (myTimer.timeChecker()) {
                break;
            }
        }
        rackMotorF.setPower(0);

        // -- turn right to line back up with the crater --
        turnRight(25, 0.3);
    }

    public void turnRight(double degrees, double power) {
        gyro.resetWithDirection(Gyro.RIGHT);

        // start the motors turning right
        double p = -1 * power;
        leftMotorF.setPower(p);
        leftMotorB.setPower(p);
        rightMotorF.setPower(p);
        rightMotorB.setPower(p);

        // loop until the robot turns :) degrees
        double d = -1 * degrees;
        while (opMode.opModeIsActive()) {
            if (gyro.getAngle() <= d) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

    }

    public void turnLeft(double degrees, double power) {
        gyro.resetWithDirection(Gyro.LEFT);

        // start the motors turning right
        double p = power;
        leftMotorF.setPower(p);
        leftMotorB.setPower(p);
        rightMotorF.setPower(p);
        rightMotorB.setPower(p);

        // loop until the robot turns :) degrees
        double d = degrees;
        while (opMode.opModeIsActive()) {
            if (gyro.getAngle() >= d) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

    }

    public void goForward(double power, double distance){
        double p = -1 * power;

        leftMotorF.setPower(p);
        leftMotorB.setPower(p);
        rightMotorF.setPower(power);
        rightMotorB.setPower(power);

        myTimer.setCompareTime(inchesToTime(distance, power));
        myTimer.start();
        while (opMode.opModeIsActive()) {
            // TODO: need brackets for the if and a break statement
            if (myTimer.timeChecker()) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }

    public void forwardToWall(double wallDistance, double power){
        double p = -1 * power;

        leftMotorF.setPower(p);
        leftMotorB.setPower(p);
        rightMotorF.setPower(power);
        rightMotorB.setPower(power);

        while (opMode.opModeIsActive()) {
            // TODO: need brackets for the if and a break statement
            if (rangeSensor.getDistance(DistanceUnit.INCH) <= wallDistance) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }

    public void dropIcon() {
        iconServo.setPosition(1);
        iconServo.setPosition(0);
        iconServo.setPosition(1);
        iconServo.setPosition(0);
    }

    public long inchesToTime(double inches, double power) {
        return (long) (0.0384 * inches * 0.5 / power);
    }

}
//                                                                                                        :D