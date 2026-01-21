package org.firstinspires.ftc.teamcode.utils;

import android.provider.Settings;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Targeting {
    MultipleTelemetry TELE;

    double unitConversionFactor = 1.02;

    int tileSize = 24; //inches

    public int robotGridX, robotGridY = 0;

    public static class Settings {
        public double flywheelRPM = 0.0;
        public double hoodAngle = 0.0;

        public Settings (double flywheelRPM, double hoodAngle) {
            this.flywheelRPM = flywheelRPM;
            this.hoodAngle = hoodAngle;
        }
    }

    // Known settings discovered using shooter test.
    // Keep the fidelity at 1 floor tile for now but we could also half it if more
    // accuracy is needed.
    public static final Settings[][] KNOWNTARGETING;
    static {
        KNOWNTARGETING = new Settings[6][6];
        // ROW 0 - Closet to the goals
        KNOWNTARGETING[0][0] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[0][1] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[0][2] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[0][3] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[0][4] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[0][5] = new Settings (4500.0, 0.1);
        // ROW 1
        KNOWNTARGETING[1][0] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[1][1] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[1][2] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[1][3] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[1][4] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[1][5] = new Settings (4500.0, 0.1);
        // ROW 2
        KNOWNTARGETING[2][0] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[2][1] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[2][2] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[2][3] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[2][4] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[2][5] = new Settings (4500.0, 0.1);
        // ROW 3
        KNOWNTARGETING[3][0] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[3][1] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[3][2] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[3][3] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[3][4] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[3][5] = new Settings (4500.0, 0.1);
        // ROW 4
        KNOWNTARGETING[4][0] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[4][1] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[4][2] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[4][3] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[4][4] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[4][5] = new Settings (4500.0, 0.1);
        // ROW 1
        KNOWNTARGETING[5][0] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[5][1] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[5][2] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[5][3] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[5][4] = new Settings (4500.0, 0.1);
        KNOWNTARGETING[5][5] = new Settings (4500.0, 0.1);
    }

    public Targeting()
    {
    }

    public Settings calculateSettings(double robotX, double robotY, double robotHeading, double robotVelocity) {
        Settings recommendedSettings = new Settings(0.0, 0.0);

        // Convert robot coordinates to inches
        double robotInchesX = robotX * unitConversionFactor;
        double robotInchesY = robotY * unitConversionFactor;

        // Find approximate location in the grid
        robotGridX = Math.floorDiv((int) robotInchesX, tileSize);
        robotGridY = Math.floorDiv((int) robotInchesY, tileSize);

        // Use Grid Location to perform lookup
        // Keep it simple for now but may want to interpolate results
        recommendedSettings.flywheelRPM = KNOWNTARGETING[robotGridY][robotGridX].flywheelRPM;
        recommendedSettings.hoodAngle = KNOWNTARGETING[robotGridY][robotGridX].hoodAngle;

        return recommendedSettings;
    }
}
