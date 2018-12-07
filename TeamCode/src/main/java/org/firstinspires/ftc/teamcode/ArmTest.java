package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class ArmTest extends LinearOpMode {
    private DcMotor leftMotorB;
    private DcMotor leftMotorF;
    private DcMotor rightMotorB;
    private DcMotor rightMotorF;

    @Override
    public void runOpMode() {

        // this is where were kinda connecting the dots between the variable and the real life motors
        leftMotorB = hardwareMap.get(DcMotor.class, "motor0");
        leftMotorF = hardwareMap.get(DcMotor.class, "motor1");
        rightMotorF = hardwareMap.get(DcMotor.class, "motor2");
        rightMotorB = hardwareMap.get(DcMotor.class, "motor3");

        //stops movement of robot quickly.
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ------------ Start of IMU Setup Stuff --------------//
        // Get the IMU and set its start up parameters

        BNO055IMU imu =  hardwareMap.get(BNO055IMU.class, "imu");

        // Now initialize it with the parameters - this starts the calibration
        telemetry.addData("Gyro Mode:", "calibrating...");
        telemetry.update();


        // let the robot sit still while calibrating the imu


        // wrap the IMU in our Gyro class so it is easier to get degrees turned
        Gyro gyro = new Gyro(imu);
        gyro.StartGyro();
        // ------------ End of IMU Stuff --------------//

        // Send a message to the drivers phone that the variables are all set.
        telemetry.addData("Gyro Status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // program waits until the start button is pressed on the phone.
        waitForStart();

        // Rotate 180 degrees
        // First start the motors turning LEFT
        //leftMotorB.setPower(0.2); // - forward
        //leftMotorF.setPower(0.2); // - forward
        //rightMotorB.setPower(0.2); // + forward
        //rightMotorF.setPower(0.2); // + forward

        // First start the motors turning RIGHT
        leftMotorB.setPower(-0.2); // - forward
        leftMotorF.setPower(-0.2); // - forward
        rightMotorB.setPower(-0.2); // + forward
        rightMotorF.setPower(-0.2); // + forward


        // Note: since this loop will run until the turn is complete we need to check if the
        //       driver pressed stop while we are turning - the opModeIsActive() check.
        gyro.resetWithDirection(-1); // left = -1, right = +1
        while(gyro.getAngle() > -200 && opModeIsActive()) { // left is positive deg, right is neg deg
            // do nothing to the motors... let them keep running

            // Let the user see the gyro degree reading on the phone
            telemetry.addData("GyroZ:", gyro.getAngle());
            telemetry.update();
        }

        // The sensor read 180 degrees. Stop the motors
        leftMotorB.setPower(0);
        leftMotorF.setPower(0);
        rightMotorB.setPower(0);
        rightMotorF.setPower(0);

    }
}
