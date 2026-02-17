package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MeasuringLoopTimes {
    private ElapsedTime elapsedtime;
    private double minLoopTime = 999999999999.0;

    private double maxLoopTime = 0.0;
    double mainLoopTime = 0.0;

    private double MeasurementStart = 0.0;
    double currentTime = 0.0;

    private double avgLoopTime = 0.0;
    private int avgLoopTimeTicker = 0;
    private double avgLoopTimeSum = 0;

    private double getTimeSeconds ()
    {
        return (double) System.currentTimeMillis()/1000.0;
    }

    public void init() {
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        MeasurementStart = getTimeSeconds();
    }


    public double getAvgLoopTime() {
        return avgLoopTime;
    }

    public double getMaxLoopTimeOneMin() {
        return maxLoopTime;
    }

    public double getMinLoopTimeOneMin() {
        return minLoopTime;
    }

    public void loop() {
        currentTime = getTimeSeconds();
        if ((MeasurementStart + 15.0) < currentTime)
        {
            minLoopTime = 9999999.0;
            maxLoopTime = 0.0;
            MeasurementStart = currentTime;

            avgLoopTime = avgLoopTimeSum / (double) avgLoopTimeTicker;
            avgLoopTimeSum = 0.0;
            avgLoopTimeTicker = 0;
        }

        mainLoopTime = elapsedtime.milliseconds();
        elapsedtime.reset();

        avgLoopTimeSum += mainLoopTime;
        avgLoopTimeTicker++;
        minLoopTime = Math.min(minLoopTime,mainLoopTime);
        maxLoopTime = Math.max(maxLoopTime,mainLoopTime);
    }

}
