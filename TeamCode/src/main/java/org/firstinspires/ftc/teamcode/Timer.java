package org.firstinspires.ftc.teamcode;

public class Timer {
    long startTime;
    long compareTime;

    public void start() {
        startTime = System.currentTimeMillis();
    }
    public boolean timeChecker() {
        return System.currentTimeMillis() - startTime >= compareTime;
    }
    public void setCompareTime(long time) {
        compareTime = time;
    }

}
