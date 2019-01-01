package org.firstinspires.ftc.teamcode.vision;

public class ColorRange {

    private double[] upper;
    private double[] lower;

    public ColorRange() {
        super();

        upper = new double[3];
        lower = new double[3];
    }

    public double[] getUpper() {
        return upper;
    }

    public double[] getLower() {
        return lower;
    }

    public void setUpper(double v1, double v2, double v3) {
        upper[0] = v1;
        upper[1] = v2;
        upper[2] = v3;
    }

    public void setLower(double v1, double v2, double v3) {
        lower[0] = v1;
        lower[1] = v2;
        lower[2] = v3;
    }
}
