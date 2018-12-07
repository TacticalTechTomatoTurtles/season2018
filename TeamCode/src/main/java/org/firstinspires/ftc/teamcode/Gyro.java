package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    BNO055IMU gyro;
    int direction = 0;
    float adjusted = 0;
    float lastAngle = 0;
    LinearOpMode opMode;
    public static int RIGHT = -1;
    public static int LEFT = 1;


    public Gyro(BNO055IMU gyro, LinearOpMode opMode) {
        this.gyro = gyro;

        // pass in the linearopmode so we can send back telemetry
        this.opMode = opMode;
    }

    public void resetWithDirection(int dir) {
        this.direction = dir;

        // reset the overall measured angle to 0
        this.adjusted = 0;

        // when we reset we need to set the last angle used to find the difference between
        // measurements to the current reading of the imu gyro sensor
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.lastAngle = angles.firstAngle;
    }

    public int getAngle() {
        // Rotating left the imu goes from 0 degrees to 180 degrees and then switches
        // to -180 degrees and then back down to 0 degrees.
        // This is tricky to keep track of so instead we find the difference between the last degree
        // measurement and the current degree measurement, and then add this difference to the
        // variable used to track the overall measured angle ('adjusted').
        // When we cross 180 degrees or 0 degrees between measurements special math is needed
        // because of the sign flipping.

        // Get the current degree measurement from the imu
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(direction == LEFT) {
            // robot is turning left
            float difference = 0;
            if(lastAngle < 0 && angles.firstAngle >= 0) {
                // rotated left past 0: degrees between measurements = the angle above 0 + the angle below 0
                // Since lastAngle is negative current - (-last) is the same as current + last
                 difference = angles.firstAngle - lastAngle;
            } else if (lastAngle > 0 && angles.firstAngle <= 0) {
                // rotated left past 180: degrees between measurements =  the angle below 180 + the angle above 180
                difference = (180 + angles.firstAngle) + (180 - lastAngle);
            } else if (lastAngle >= 0 && angles.firstAngle >= 0) {
                difference = angles.firstAngle - lastAngle;
            }  else if (lastAngle <= 0 && angles.firstAngle <= 0) {
                difference = angles.firstAngle - lastAngle;
            }

            // Now that we have the difference in degrees between measurements add that to the overall angle
            // Rotating left makes positive angles so add the difference to the overall
            adjusted = adjusted + difference;
        } else {
            // right
            float difference = 0;
            if(lastAngle < 0 && angles.firstAngle >= 0) {
                // rotated right past 180:  degrees between measurements =  the angle above 180 + the angle below 180
                difference = (180 - angles.firstAngle) + (180 + lastAngle);
            } else if (lastAngle > 0 && angles.firstAngle <= 0) {
                // rotated right past 0: degrees between measurements = the angle above 0 + the angle below 0
                // Since current is negative last - (-current) is the same as last + current
                difference = lastAngle - angles.firstAngle;
            } else if (lastAngle >= 0 && angles.firstAngle >= 0) {
                difference =  lastAngle - angles.firstAngle;
            }  else if (lastAngle <= 0 && angles.firstAngle <= 0) {
                difference = lastAngle - angles.firstAngle;
            }

            // Now that we have the difference in degrees between measurements add that to the overall angle
            // Rotating right makes negative angles so subtract the difference from the overall
            adjusted = adjusted - difference;

        }

        // Update the last measurement to the current measurement so this can be repeated on the next call
        lastAngle = angles.firstAngle;

        // send the current overall angle to the phone
        opMode.telemetry.addData("Gyro angle:", adjusted);
        opMode.telemetry.update();

        // return the current overall angle
        return (int) adjusted;
    }

    public void StartGyro(){
        // The imu complains if you try to initialize it twice so check if it is already initialized
        if(!gyro.isGyroCalibrated()) {

            // Set the parameters to initialize the imu
            // This was referenced from http://stemrobotics.cs.pdx.edu/node/7265
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // send a message to the phone that the gyro is being initialized
            opMode.telemetry.addData("Gyro Mode:", "calibrating...");
            opMode.telemetry.update();

            // initialize the imu with the parameters
            gyro.initialize(parameters);

            // wait for rhe calibration to complete
            while (!gyro.isGyroCalibrated()) {
                // do noting... just wait
                opMode.idle();
            }
        }

        // send a message to the phone about the calibration status
        opMode.telemetry.addData("Gyro Status", gyro.getCalibrationStatus().toString());
        opMode.telemetry.update();
    }
}
