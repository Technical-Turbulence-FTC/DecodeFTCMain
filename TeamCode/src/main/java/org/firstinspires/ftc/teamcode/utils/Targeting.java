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

    public Settings calculateSettings(double robotX, double robotY, double robotHeading, double robotVelocity, boolean interpolate) {
        Settings recommendedSettings = new Settings(0.0, 0.0);
    
        // Convert robot coordinates to inches
        double robotInchesX = robotX * unitConversionFactor;
        double robotInchesY = robotY * unitConversionFactor;
    
        // Find approximate location in the grid
        int gridX = Math.floorDiv((int) robotInchesX, tileSize);
        int gridY = Math.floorDiv((int) robotInchesY, tileSize);
    
        // clamp
        gridX = Math.max(0, Math.min(gridX, KNOWNTARGETING[0].length - 1));
        gridY = Math.max(0, Math.min(gridY, KNOWNTARGETING.length - 1));
    
        if (!interpolate) {
            // simple mapping (as before)
            recommendedSettings.flywheelRPM = KNOWNTARGETING[gridY][gridX].flywheelRPM;
            recommendedSettings.hoodAngle = KNOWNTARGETING[gridY][gridX].hoodAngle;
            return recommendedSettings;
        }
    
        /*
            -- TEST without interpolation first.
            -- THIS USES BILINEAR INTERPOLATION (most accurate)
                -- takes four neighbors and finds the weighted average.
        */
        
        int x0 = gridX;
        int x1 = Math.min(x0 + 1, KNOWNTARGETING[0].length - 1);
        int y0 = gridY;
        int y1 = Math.min(y0 + 1, KNOWNTARGETING.length - 1);
    
        // inside tile
        double tx = (robotInchesX - x0 * tileSize) / tileSize;
        double ty = (robotInchesY - y0 * tileSize) / tileSize;
    
        // flywheel interpolation
        double f00 = KNOWNTARGETING[y0][x0].flywheelRPM;
        double f10 = KNOWNTARGETING[y0][x1].flywheelRPM;
        double f01 = KNOWNTARGETING[y1][x0].flywheelRPM;
        double f11 = KNOWNTARGETING[y1][x1].flywheelRPM;
        double flyRPM = f00 * (1 - tx) * (1 - ty) +
                        f10 * tx * (1 - ty) +
                        f01 * (1 - tx) * ty +
                        f11 * tx * ty;
    
        // hood interpolation
        double h00 = KNOWNTARGETING[y0][x0].hoodAngle;
        double h10 = KNOWNTARGETING[y0][x1].hoodAngle;
        double h01 = KNOWNTARGETING[y1][x0].hoodAngle;
        double h11 = KNOWNTARGETING[y1][x1].hoodAngle;
        double hood = h00 * (1 - tx) * (1 - ty) +
                      h10 * tx * (1 - ty) +
                      h01 * (1 - tx) * ty +
                      h11 * tx * ty;
    
        recommendedSettings.flywheelRPM = flyRPM;
        recommendedSettings.hoodAngle = hood;
    
        return recommendedSettings;
    }

}
