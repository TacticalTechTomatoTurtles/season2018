package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.MineralIdentifier;
import org.firstinspires.ftc.teamcode.vision.Result;

import java.util.List;

@Autonomous
public class BlueFluffyUnicorns extends LinearOpMode {
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
        leftServoF = hardwareMap.get(Servo.class, "iconDropServo");
        Tanner = hardwareMap.get(TouchSensor.class, "touchSensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        MineralIdentifier minid = new MineralIdentifier();
        Gyro gyro = new Gyro(imu, this);
        myTimer = new Timer();
        iconServo = hardwareMap.get(Servo.class, "alex");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        //stops movement of robot quickly.
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rackMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        minid.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        // start the vision system
        minid.enable();

        // wait for the start button to be pushed
        waitForStart();

        myTimer.setCompareTime(2300);
        rackMotorF.setPower(-1);
        myTimer.start();
        while (opModeIsActive()) {
            if (myTimer.timeChecker()) {
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
        leftMotorF.setPower(0.5);
        leftMotorB.setPower(0.5);
        rightMotorF.setPower(0.5);
        rightMotorB.setPower(0.5);

        // loop until the robot turns 25 degrees
        while (opModeIsActive()) {
            if (gyro.getAngle() >= 25) {
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
            if (myTimer.timeChecker()) {
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
            if (gyro.getAngle() <= -25) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

        // ASSERT! we are all detached from the lander

        //Starts looking for mineral
        while (opModeIsActive()) {
            List<Result> results = minid.getResults();
            log("results: " + (results == null ? "null" : results.size()));

            // see if we got a hit from the vision system
            if (results != null && results.size() == 1) {
                Result item = results.get(0);
                double width = item.getFrameSize().getWidth();
                double length = item.getFrameSize().getHeight();
                // double first = width / 3;
                //  double second = first * 2;
                double half = width / 2;
                //double lengthDivider = length / 2;

                if (item.getCenter().x < half) {
                    // CHECK: block is in front = go forward 18 inches
                    leftMotorF.setPower(-0.5);
                    leftMotorB.setPower(-0.5);
                    rightMotorF.setPower(0.5);
                    rightMotorB.setPower(0.5);

                    myTimer.setCompareTime(inchesToTime(18));
                    myTimer.start();
                    while (opModeIsActive()) {
                        // TODO: need brackets for the if and a break statement
                        if (myTimer.timeChecker()) {
                            break;
                        }
                    }

                    leftMotorF.setPower(0);
                    leftMotorB.setPower(0);
                    rightMotorF.setPower(0);
                    rightMotorB.setPower(0);

                    // CHECK: go forward until 5 inches from wall
                    leftMotorF.setPower(-0.25);
                    leftMotorB.setPower(-0.25);
                    rightMotorF.setPower(0.25);
                    rightMotorB.setPower(0.25);

                    myTimer.setCompareTime(inchesToTime(50));
                    myTimer.start();
                    while (opModeIsActive()) {
                        if (myTimer.timeChecker()) {
                            break;
                        }
                    }

                    leftMotorF.setPower(0);
                    leftMotorB.setPower(0);
                    rightMotorF.setPower(0);
                    rightMotorB.setPower(0);

                    BaseGyroSet = 15;
                    break;

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
                        if (gyro.getAngle() <= -40) {
                            break;
                        }
                    }

                    // start the motors turning right
                    // CHECK: go forward?
                    myTimer.setCompareTime(inchesToTime(18));
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

                    leftMotorF.setPower(0);
                    leftMotorB.setPower(0);
                    rightMotorF.setPower(0);
                    rightMotorB.setPower(0);

                    gyro.resetWithDirection(Gyro.LEFT);

                    // start the motors turning right
                    leftMotorF.setPower(0.2);
                    leftMotorB.setPower(0.2);
                    rightMotorF.setPower(0.2);
                    rightMotorB.setPower(0.2);

                    // loop until the robot turns 25 degrees
                    while (opModeIsActive()) {
                        if (gyro.getAngle() >= 35) {
                            break;
                        }
                    }

                    leftMotorF.setPower(0);
                    leftMotorB.setPower(0);
                    rightMotorF.setPower(0);
                    rightMotorB.setPower(0);

                    leftMotorF.setPower(-0.25);
                    leftMotorB.setPower(-0.25);
                    rightMotorF.setPower(0.25);
                    rightMotorB.setPower(0.25);

                    myTimer.setCompareTime(inchesToTime(20));
                    myTimer.start();
                    while (opModeIsActive()) {
                        if (myTimer.timeChecker()) {
                            break;
                        }
                    }

                    leftMotorF.setPower(0);
                    leftMotorB.setPower(0);
                    rightMotorF.setPower(0);
                    rightMotorB.setPower(0);

                    gyro.resetWithDirection(Gyro.LEFT);

                    leftMotorF.setPower(-0.25);
                    leftMotorB.setPower(-0.25);
                    rightMotorF.setPower(-0.25);
                    rightMotorB.setPower(-0.25);

                    while (opModeIsActive()) {
                        if (gyro.getAngle() >= 17) {
                            break;
                        }
                    }

                    leftMotorF.setPower(-0.25);
                    leftMotorB.setPower(-0.25);
                    rightMotorF.setPower(0.25);
                    rightMotorB.setPower(0.25);

                   myTimer.setCompareTime(inchesToTime(20));
                   myTimer.start();
                   while (opModeIsActive()) {
                       if (myTimer.timeChecker()) {
                           break;
                       }
                   }

                    leftMotorF.setPower(0);
                    leftMotorB.setPower(0);
                    rightMotorF.setPower(0);
                    rightMotorB.setPower(0);

                    BaseGyroSet = 60;
                    break;
                }
                // navigate to the item
            } else {
                // go left
                gyro.resetWithDirection(Gyro.LEFT);

                // start the motors turning right
                leftMotorF.setPower(0.2);
                leftMotorB.setPower(0.2);
                rightMotorF.setPower(0.2);
                rightMotorB.setPower(0.2);

                // loop until the robot turns 25 degrees
                while (opModeIsActive()) {
                    if (gyro.getAngle() >= 41) {
                        break;
                    }
                }

                leftMotorF.setPower(0);
                leftMotorB.setPower(0);
                rightMotorF.setPower(0);
                rightMotorB.setPower(0);

                myTimer.setCompareTime(inchesToTime(18));
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

                leftMotorF.setPower(0);
                leftMotorB.setPower(0);
                rightMotorF.setPower(0);
                rightMotorB.setPower(0);

                gyro.resetWithDirection(Gyro.RIGHT);

                // start the motors turning right
                leftMotorF.setPower(-0.2);
                leftMotorB.setPower(-0.2);
                rightMotorF.setPower(-0.2);
                rightMotorB.setPower(-0.2);

                // loop until the robot turns 25 degrees
                while (opModeIsActive()) {
                    if (gyro.getAngle() <= -35) {
                        break;
                    }
                }

                leftMotorF.setPower(0);
                leftMotorB.setPower(0);
                rightMotorF.setPower(0);
                rightMotorB.setPower(0);

                leftMotorF.setPower(-0.25);
                leftMotorB.setPower(-0.25);
                rightMotorF.setPower(0.25);
                rightMotorB.setPower(0.25);

                myTimer.setCompareTime(inchesToTime(20));
                myTimer.start();
                while (opModeIsActive()) {
                    if (myTimer.timeChecker()) {
                        break;
                    }
                }

                gyro.resetWithDirection(Gyro.LEFT);

                while (opModeIsActive()) {
                    if (gyro.getAngle() >= -17) {
                        break;
                    }
                }

                leftMotorF.setPower(0);
                leftMotorB.setPower(0);
                rightMotorF.setPower(0);
                rightMotorB.setPower(0);

                BaseGyroSet = 85;
                break;
            }
        }
        // end of looking for mineral

        // disables the Mineral Identifier
        minid.disable();

        log("exiting.");
        // drops the icon, if it doesn't work shakes off icon
        iconServo.setPosition(1);
        iconServo.setPosition(0);
        iconServo.setPosition(1);
        iconServo.setPosition(0);

        leftMotorF.setPower(0.2);
        leftMotorB.setPower(0.2);
        rightMotorF.setPower(0.2);
        rightMotorB.setPower(0.2);
        //TODO is mr.andy correct? NO
        gyro.resetWithDirection(Gyro.RIGHT);
        // sets the angle to make the robot be equal to the base so we can back into crater

        while (opModeIsActive()) {
            if (gyro.getAngle() <= -BaseGyroSet) {
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

        leftMotorF.setPower(-0.2);
        leftMotorB.setPower(-0.2);
        rightMotorF.setPower(0.2);
        rightMotorB.setPower(0.2);

        myTimer.setCompareTime(inchesToTime(160));
        leftMotorF.setPower(0.25);
        leftMotorB.setPower(0.25);
        rightMotorF.setPower(-0.25);
        rightMotorB.setPower(-0.25);

        myTimer.start();
        while (opModeIsActive()) {
            if (myTimer.timeChecker()){
                break;
            }
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        // in crater
    }

    private void log(String message) {
        telemetry.addData("log", message);
        telemetry.update();
    }
    public long inchesToTime(double inches) {
        return (long) (0.0384 * inches * 1000.0);
    }
}
