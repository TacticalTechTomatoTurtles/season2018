package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    BNO055IMU gyro;
    int direction = 0;
    float adjusted = 0;
    float lastAngle = 0;

    public Gyro(BNO055IMU gyro) {
        this.gyro = gyro;
    }

    public void resetWithDirection(int dir) {
        this.direction = dir;
        this.adjusted = 0;
    }

    public int getAngle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(direction >= 0) {
            // left
            float difference = 0;
            if(lastAngle < 0 && angles.firstAngle >= 0) {
                // rotated left past 0
                 difference = angles.firstAngle - lastAngle;
            } else if (lastAngle > 0 && angles.firstAngle <= 0) {
                // rotated left past 180
                difference = (180 + angles.firstAngle) + (180 - lastAngle);
            } else if (lastAngle >= 0 && angles.firstAngle >= 0) {
                difference = angles.firstAngle - lastAngle;
            }  else if (lastAngle <= 0 && angles.firstAngle <= 0) {
                difference = angles.firstAngle - lastAngle;
            }
            adjusted = adjusted + difference;
        } else {
            // right
            float difference = 0;
            if(lastAngle < 0 && angles.firstAngle >= 0) {
                // rotated right past 180
                difference = (180 - angles.firstAngle) + (180 + lastAngle);
            } else if (lastAngle > 0 && angles.firstAngle <= 0) {
                // rotated right past 0
                difference = lastAngle - angles.firstAngle;
            } else if (lastAngle >= 0 && angles.firstAngle >= 0) {
                difference =  lastAngle - angles.firstAngle;
            }  else if (lastAngle <= 0 && angles.firstAngle <= 0) {
                difference = lastAngle - angles.firstAngle;
            }
            adjusted = adjusted - difference;

        }

        lastAngle = angles.firstAngle;

        return (int) adjusted;
    }
}
