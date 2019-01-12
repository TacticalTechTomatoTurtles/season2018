package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.vision.MineralIdentifier;
import org.firstinspires.ftc.teamcode.vision.Result;

import java.util.List;

@Autonomous
public class CoordanatePlaneHalf extends LinearOpMode {

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

        public void runOpMode() {

            // get all the hardware
            leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
            leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
            rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
            rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
            armMotorF = hardwareMap.get(DcMotor.class, "motor4");
            rackMotorF = hardwareMap.get(DcMotor.class, "motorX");
            rightServoF = hardwareMap.get(Servo.class, "steve");
            leftServoF = hardwareMap.get(Servo.class, "iconDropServo");
            Tanner = hardwareMap.get(TouchSensor.class, "touchSensor");
            BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
            Gyro gyro = new Gyro(imu, this);
            myTimer = new Timer();
            iconServo = hardwareMap.get(Servo.class, "alex");

            //stops movement of robot quickly.
            leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            MineralIdentifier minid = new MineralIdentifier();

            // can replace with ActivityViewDisplay.getInstance() for fullscreen
            minid.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

            // start the vision system
            minid.enable();
            log("waiting for start");
            gyro.StartGyro();
            waitForStart();


            while (opModeIsActive()) {
                List<Result> results = minid.getResults();
                log("results: " + (results == null ? "null" : results.size()));

                if (results != null && results.size() == 1) {
                    Result item = results.get(0);
                    double width = item.getFrameSize().getWidth();
                   // double first = width / 3;
                  //  double second = first * 2;
                    double half = width / 2;

                    if (item.getCenter().x < half) {
                        //turn left
                        myTimer.setCompareTime(inchesToTime(1));
                        leftMotorF.setPower(-0.5);
                        leftMotorB.setPower(-0.5);
                        rightMotorF.setPower(0.5);
                        rightMotorB.setPower(0.5);
                        myTimer.start();
                        while (opModeIsActive()) {
                            if (myTimer.timeChecker()) {
                                break;
                            }
                        }
                    } else if (item.getCenter().x > half) {
                        //turn right
                        gyro.resetWithDirection(Gyro.RIGHT);

                        // start the motors turning right
                        leftMotorF.setPower(-0.2);
                        leftMotorB.setPower(-0.2);
                        rightMotorF.setPower(-0.2);
                        rightMotorB.setPower(-0.2);
                        // loop until the robot turns 25 degrees
                        while (opModeIsActive()) {
                            if (gyro.getAngle() <= -5) {
                                break;
                            }
                        }
                    }
                    // navigate to the item
                }else {
                    // go forward
                    gyro.resetWithDirection(Gyro.LEFT);

                    leftMotorF.setPower(0.2);
                    leftMotorB.setPower(0.2);
                    rightMotorF.setPower(0.2);
                    rightMotorB.setPower(0.2);
                    while (opModeIsActive()) {
                        if (gyro.getAngle() >= 5) {
                            break;
                        }
                    }
                }
                leftMotorF.setPower(0);
                leftMotorB.setPower(0);
                rightMotorF.setPower(0);
                rightMotorB.setPower(0);
            }


            minid.disable();
            log("exiting.");
        }

        private void log(String message) {
            telemetry.addData("log", message);
            telemetry.update();
        }

        public long inchesToTime(double inches) {
            return (long) (0.0384 * inches * 1000.0);
        }
    }
