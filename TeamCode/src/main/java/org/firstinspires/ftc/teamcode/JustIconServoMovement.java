package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.vision.MineralIdentifier;

@Autonomous
public class JustIconServoMovement extends LinearOpMode {

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
    private int BaseGyroSet;

    @Override
    public void runOpMode() {
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        rackMotorF = hardwareMap.get(DcMotor.class, "motorX");
        rightServoF = hardwareMap.get(Servo.class, "steve");
        //leftServoF = hardwareMap.get(Servo.class, "iconDropServo");
        Tanner = hardwareMap.get(TouchSensor.class, "touchSensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        MineralIdentifier minid = new MineralIdentifier();
        Gyro gyro = new Gyro(imu, this);
        myTimer = new Timer();
        iconServo = hardwareMap.get(Servo.class, "iconDropServo");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        waitForStart();

        while (opModeIsActive()) {

            iconServo.setPosition(0);
            iconServo.setPosition(2);
        }
    }
}