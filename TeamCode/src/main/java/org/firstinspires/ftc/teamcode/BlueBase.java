package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
public class BlueBase extends LinearOpMode {

    private Timer myTimer;
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;
    private DcMotor armMotorF;
    private Servo rightServoF;
    private TouchSensor touchSensor;

    private int LOOKING_FOR_WALL = 0;
    private int TURNING_TO_BASE = 1;
    private int DRIVE_TO_BASE = 2;
    private int BACK_UP_FROM_BASE = 3;
    private int DROP_ICON = 4;
    private int BACK_UP_FULLY = 5;


    @Override
    public void runOpMode() {
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");
        armMotorF = hardwareMap.get(DcMotor.class, "motor4");
        rightServoF = hardwareMap.get(Servo.class, "steve");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");


        waitForStart();
        double tgtPowerLB = 0;
        double tgtPowerRB = 0;
        double tgtPowerLF = 0;
        double tgtPowerRF = 0;
        double tgtPowerArm = 0;

        int state = LOOKING_FOR_WALL;

        while (opModeIsActive()) {
            // go forward until you hit the wall
            if (state == LOOKING_FOR_WALL) {
                boolean found = findWall();
                if (found) {
                    state = TURNING_TO_BASE;
                    myTimer.setCompareTime(2000);
                    myTimer.start();
                }
            } else if (state == TURNING_TO_BASE) {
                if (turning()) {
                    state = DRIVE_TO_BASE;
                }

            }else if (state == DRIVE_TO_BASE) {
                boolean found = findWall();
                if (found) {
                    state = BACK_UP_FROM_BASE;
                    myTimer.setCompareTime(1000);
                    myTimer.start();
                }
            }else if (state == BACK_UP_FROM_BASE) {
                        if (backUp()) {
                            state = DROP_ICON;
                            myTimer.setCompareTime(1000);
                            myTimer.start();
                        }
            } else if (state == DROP_ICON) {
                            if (iconDrop()) {
                                state = BACK_UP_FULLY;
                            }
            } else if (state == BACK_UP_FULLY) {
                                if (backUpFully()) {
                                    myTimer.setCompareTime(15000);
                                    myTimer.start();
                                }

                        }


            //else if (state === ) {

            //}
            // turn right 90 degrees

            // go forward y inches

            // drop the figure

            //back into the crater
        }

        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }

    private boolean findWall() {
        // is sensor pressed
        if (touchSensor.isPressed()) {
            return true;
        } else {
            // if no go forward at speed x and return false
            leftMotorF.setPower(1);
            leftMotorB.setPower(1);
            rightMotorF.setPower(1);
            rightMotorB.setPower(1);
            return false;
        }
    }

    private boolean turning() {
        if (myTimer.timeChecker()) {
            return true;
        } else {
            leftMotorB.setPower(1);
            leftMotorF.setPower(1);
            rightMotorB.setPower(-1);
            rightMotorF.setPower(-1);
            return false;
        }
    }
    private boolean backUp() {
        if (myTimer.timeChecker()) {
            leftMotorB.setPower(0);
            leftMotorF.setPower(0);
            rightMotorB.setPower(0);
            rightMotorF.setPower(0);
            return true;
        }else {
            leftMotorB.setPower(-1);
            leftMotorF.setPower(-1);
            rightMotorB.setPower(-1);
            rightMotorF.setPower(-1);
            return false;
        }
    }
    private boolean iconDrop() {
        if (myTimer.timeChecker()) {
            armMotorF.setPower(1);
            return true;
        }else {
            return false;
        }
    }
        private boolean backUpFully() {
            if (myTimer.timeChecker()) {
                leftMotorB.setPower(0);
                leftMotorF.setPower(0);
                rightMotorB.setPower(0);
                rightMotorF.setPower(0);
                return true;
            }else {
                leftMotorB.setPower(-1);
                leftMotorF.setPower(-1);
                rightMotorB.setPower(-1);
                rightMotorF.setPower(-1);
                return false;
            }
        }


    }

