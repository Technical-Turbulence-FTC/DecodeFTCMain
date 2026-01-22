package org.firstinspires.ftc.teamcode.utils;

import android.provider.Settings;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Targeting {
    MultipleTelemetry TELE;

    double cancelOffsetX = 7.071067811;
    double cancelOffsetY = 7.071067811;
    double unitConversionFactor = 0.95;

    int tileSize = 24; //inches

    public double robotInchesX, robotInchesY = 0.0;

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
        KNOWNTARGETING[0][0] = new Settings (3000.0, 0.25);
        KNOWNTARGETING[0][1] = new Settings (3001.0, 0.25);
        KNOWNTARGETING[0][2] = new Settings (3002.0, 0.25);
        KNOWNTARGETING[0][3] = new Settings (3302.0, 0.2);
        KNOWNTARGETING[0][4] = new Settings (3503.0, 0.15);
        KNOWNTARGETING[0][5] = new Settings (3505.0, 0.15);
        // ROW 1
        KNOWNTARGETING[1][0] = new Settings (3010.0, 0.25);
        KNOWNTARGETING[1][1] = new Settings (3011.0, 0.25);
        KNOWNTARGETING[1][2] = new Settings (3012.0, 0.25);
        KNOWNTARGETING[1][3] = new Settings (3313.0, 0.15);
        KNOWNTARGETING[1][4] = new Settings (3514.0, 0.15);
        KNOWNTARGETING[1][5] = new Settings (3515.0, 0.15);
        // ROW 2
        KNOWNTARGETING[2][0] = new Settings (3020.0, 0.1);
        KNOWNTARGETING[2][1] = new Settings (3000.0, 0.25);
        KNOWNTARGETING[2][2] = new Settings (3000.0, 0.15);
        KNOWNTARGETING[2][3] = new Settings (3000.0, 0.15);
        KNOWNTARGETING[2][4] = new Settings (3524.0, 0.15);
        KNOWNTARGETING[2][5] = new Settings (3525.0, 0.15);
        // ROW 3
        KNOWNTARGETING[3][0] = new Settings (3030.0, 0.15);
        KNOWNTARGETING[3][1] = new Settings (3031.0, 0.15);
        KNOWNTARGETING[3][2] = new Settings (3000.0, 0.15);
        KNOWNTARGETING[3][3] = new Settings (3000.0, 0.15);
        KNOWNTARGETING[3][4] = new Settings (3000.0, 0.03);
        KNOWNTARGETING[3][5] = new Settings (3535.0, 0.1);
        // ROW 4
        KNOWNTARGETING[4][0] = new Settings (4540.0, 0.1);
        KNOWNTARGETING[4][1] = new Settings (4541.0, 0.1);
        KNOWNTARGETING[4][2] = new Settings (4542.0, 0.1);
        KNOWNTARGETING[4][3] = new Settings (4543.0, 0.1);
        KNOWNTARGETING[4][4] = new Settings (4544.0, 0.1);
        KNOWNTARGETING[4][5] = new Settings (4545.0, 0.1);
        // ROW 1
        KNOWNTARGETING[5][0] = new Settings (4550.0, 0.1);
        KNOWNTARGETING[5][1] = new Settings (4551.0, 0.1);
        KNOWNTARGETING[5][2] = new Settings (4552.0, 0.1);
        KNOWNTARGETING[5][3] = new Settings (4553.0, 0.1);
        KNOWNTARGETING[5][4] = new Settings (4554.0, 0.1);
        KNOWNTARGETING[5][5] = new Settings (4555.0, 0.1);
    }

    public Targeting()
    {
    }

    public Settings calculateSettings(double robotX, double robotY, double robotHeading, double robotVelocity) {
        Settings recommendedSettings = new Settings(0.0, 0.0);

        double cos45 = Math.cos(Math.toRadians(-45));
        double sin45 = Math.sin(Math.toRadians(-45));
        double rotatedY = (robotX -40.0) * sin45 + (robotY +7.0) * cos45;
        double rotatedX = (robotX -40.0) * cos45 - (robotY +7.0) * sin45;

        // Convert robot coordinates to inches
        robotInchesX = rotatedX * unitConversionFactor;
        robotInchesY = rotatedY * unitConversionFactor;

        // Find approximate location in the grid
        robotGridX = Math.abs(Math.floorDiv((int) robotInchesX, tileSize) +1);
        robotGridY = Math.abs(Math.floorDiv((int) robotInchesY, tileSize));

        // Use Grid Location to perform lookup
        // Keep it simple for now but may want to interpolate results
        if ((robotGridY < 6) && (robotGridX <6)) {
            recommendedSettings.flywheelRPM = KNOWNTARGETING[robotGridY][robotGridX].flywheelRPM;
            recommendedSettings.hoodAngle = KNOWNTARGETING[robotGridY][robotGridX].hoodAngle;
        }
        return recommendedSettings;
    }
}
