package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
public class LanderBlueBase extends LinearOpMode {
    private Timer myTimer;
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;
    private DcMotor armMotorF;
    private DcMotor rackMotorF;
    private Servo rightServoF;
    private TouchSensor Tanner;

    private int LOOKING_FOR_WALL = 1;
    private int BACK_UP_WALL = 2;
    private int TURNING_TO_BASE = 3;
    private int DRIVE_TO_BASE = 4;
    private int BACK_UP_FROM_BASE = 5;
    private int DROP_ICON = 6;
    private int BACK_UP_FULLY = 7;
    private int DETACH_LANDER = 0;
    private int TURN_AFTER_DETACH = 8;


    @Override
    public void runOpMode() {
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        //flip a motor and see what's up
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        rackMotorF = hardwareMap.get(DcMotor.class, "motorX");
        rightServoF = hardwareMap.get(Servo.class, "steve");
        Tanner = hardwareMap.get(TouchSensor.class, "touchSensor");
        BNO055IMU imu =  hardwareMap.get(BNO055IMU.class, "imu");


        waitForStart();
        double tgtPowerLB = 0;
        double tgtPowerRB = 0;
        double tgtPowerLF = 0;
        double tgtPowerRF = 0;
        double tgtPowerArm = 0;
        double tgtPowerRack = 0;
        myTimer = new Timer();
        Gyro gyro = new Gyro(imu);

        myTimer.setCompareTime(inchesToTime(4));

        int state = DETACH_LANDER;

        while (opModeIsActive()) {

        if (state == DETACH_LANDER) {
            if (detach()) {
                state = TURN_AFTER_DETACH;
                gyro.StartGyro();
                telemetry.addData("Gyro Mode:", "calibrating...");
                telemetry.update();
                myTimer.setCompareTime(inchesToTime(5));
                myTimer.start();
            }   // go forward until you hit the wall
        } else if (state == TURN_AFTER_DETACH) {
            if (turnFromDetach()) {
                state = LOOKING_FOR_WALL;
                myTimer.setCompareTime(inchesToTime(5));
                myTimer.start();
            }
        } else if (state == LOOKING_FOR_WALL) {
            boolean found = findWall();
            if (found) {
                state = BACK_UP_WALL;
                myTimer.setCompareTime(inchesToTime(5));
                myTimer.start();
            }
        } else if (state == BACK_UP_WALL) {
            if (backUpWall()) {
                state = TURNING_TO_BASE;
                myTimer.setCompareTime(700);
                myTimer.start();
            }

        } else if (state == TURNING_TO_BASE) {
            if (turningRight()) {
                state = DRIVE_TO_BASE;
            }

        } else if (state == DRIVE_TO_BASE) {
            boolean found = findWall();
            if (found) {
                state = BACK_UP_FROM_BASE;
                myTimer.setCompareTime(inchesToTime(22));
                myTimer.start();
            }
        } else if (state == BACK_UP_FROM_BASE) {
            if (backUp()) {
                state = DROP_ICON;
                myTimer.setCompareTime(2000);
                myTimer.start();
            }
        } else if (state == DROP_ICON) {
            if (iconDrop()) {
                state = BACK_UP_FULLY;
                armMotorF.setPower(.25);
            }
        } else if (state == BACK_UP_FULLY) {
            if (backUpFully()) {
                myTimer.setCompareTime(inchesToTime(100));
                myTimer.start();
            }

        }

        telemetry.addData("Is Pressed", Tanner.isPressed());

        //else if (state === ) {

        //   {
        // turn right 90 degrees

        // go forward y inches

        // drop the figure

        //back into the crater

        //     }
        //}
    }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
}
    private boolean findWall () {
        // is sensor pressed
        if (Tanner.isPressed() == true) {
            leftMotorB.setPower(0);
            leftMotorF.setPower(0);
            rightMotorB.setPower(0);
            rightMotorF.setPower(0);
            return true;
        } else {
            // if no go forward at speed x and return false
            leftMotorF.setPower(1);
            leftMotorB.setPower(1);
            rightMotorF.setPower(-1);
            rightMotorB.setPower(-1);
            return false;
        }
    }

    private boolean backUpWall () {
        if (myTimer.timeChecker()) {
            leftMotorB.setPower(0);
            leftMotorF.setPower(0);
            rightMotorB.setPower(0);
            rightMotorF.setPower(0);
            return true;
        } else {

            leftMotorB.setPower(-1);
            leftMotorF.setPower(-1);
            rightMotorB.setPower(1);
            rightMotorF.setPower(1);
            return false;
        }
    }
    private boolean turningRight () {
        if (myTimer.timeChecker()) {
            return true;
        } else {
            leftMotorB.setPower(1);
            leftMotorF.setPower(1);
            rightMotorB.setPower(1);
            rightMotorF.setPower(1);
            return false;
        }
    }


    private boolean backUp () {
        if (myTimer.timeChecker()) {
            leftMotorB.setPower(0);
            leftMotorF.setPower(0);
            rightMotorB.setPower(0);
            rightMotorF.setPower(0);
            return true;
        } else {
            leftMotorB.setPower(-1);
            leftMotorF.setPower(-1);
            rightMotorB.setPower(1);
            rightMotorF.setPower(1);
            return false;
        }
    }

    private boolean iconDrop () {
        if (myTimer.timeChecker()) {
            armMotorF.setPower(.25);
            return true;
        } else {
            return false;
        }
    }

    private boolean backUpFully () {
        if (myTimer.timeChecker()) {
            leftMotorB.setPower(0);
            leftMotorF.setPower(0);
            rightMotorB.setPower(0);
            rightMotorF.setPower(0);
            return true;
        } else {

            leftMotorB.setPower(-1);
            leftMotorF.setPower(-1);
            rightMotorB.setPower(1);
            rightMotorF.setPower(1);
            return false;
        }
    }
   private boolean detach () {
        if (myTimer.timeChecker()) {
         rackMotorF.setPower(0);
            return true;
        } else {
          rackMotorF.setPower(1);
            return false;
            }
        }
    private boolean turnFromDetach () {
        if (myTimer.timeChecker()) {
            leftMotorB.setPower(0);
            leftMotorF.setPower(0);
            rightMotorB.setPower(0);
            rightMotorF.setPower(0);
            return true;
        } else {
            leftMotorB.setPower(-1);
            leftMotorF.setPower(-1);
            rightMotorB.setPower(1);
            rightMotorF.setPower(1);
            return false;
        }
    }

    public long inchesToTime ( double inches){
        return (long) (0.0384 * inches * 1000.0);
    }
}

