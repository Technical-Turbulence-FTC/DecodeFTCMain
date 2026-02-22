package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.constants.ServoPositions;

public class Targeting {
    // Known settings discovered using shooter test.
    // Keep the fidelity at 1 floor tile for now but we could also half it if more
    // accuracy is needed.
    public static final Settings[][] KNOWNTARGETING;

    static {
        KNOWNTARGETING = new Settings[6][6];
        // ROW 0 - Closet to the goals
        KNOWNTARGETING[0][0] = new Settings(2300.0, 0.93);
        KNOWNTARGETING[0][1] = new Settings(2300.0, 0.93);
        KNOWNTARGETING[0][2] = new Settings(2500.0, 0.78);
        KNOWNTARGETING[0][3] = new Settings(2800.0, 0.68);
        KNOWNTARGETING[0][4] = new Settings(3000.0, 0.58);
        KNOWNTARGETING[0][5] = new Settings(3000.0, 0.58);
        // ROW 1
        KNOWNTARGETING[1][0] = new Settings(2300.0, 0.93);
        KNOWNTARGETING[1][1] = new Settings(2300.0, 0.93);
        KNOWNTARGETING[1][2] = new Settings(2600.0, 0.78);
//        KNOWNTARGETING[1][3] = new Settings (2800.0, 0.62);
//        KNOWNTARGETING[1][4] = new Settings (3000.0, 0.55);
//        KNOWNTARGETING[1][5] = new Settings (3200.0, 0.50);
        KNOWNTARGETING[1][3] = new Settings(2800.0, 0.68);  // Real settings replaced with (0,3) new Settings (2800.0, 0.62);
        KNOWNTARGETING[1][4] = new Settings(3000.0, 0.58);  // Real setting replaced with (0,4)  new Settings (3000.0, 0.55);
        KNOWNTARGETING[1][5] = new Settings(3200.0, 0.50);
        // ROW 2
//        KNOWNTARGETING[2][0] = new Settings (2500.0, 0.78);
//        KNOWNTARGETING[2][1] = new Settings (2500.0, 0.78);
//        KNOWNTARGETING[2][2] = new Settings (2700.0, 0.60);
//        KNOWNTARGETING[2][3] = new Settings (2900.0, 0.53);
//        KNOWNTARGETING[2][4] = new Settings (3100.0, 0.50);
//        KNOWNTARGETING[2][5] = new Settings (3100.0, 0.50);
        KNOWNTARGETING[2][0] = new Settings(2500.0, 0.78);
        KNOWNTARGETING[2][1] = new Settings(2500.0, 0.78);
        KNOWNTARGETING[2][2] = new Settings(2700.0, 0.60);
        KNOWNTARGETING[2][3] = new Settings(2800.0, 0.62);  // Real settings replaced with (1,3) new Settings (2900.0, 0.53);
        KNOWNTARGETING[2][4] = new Settings(3000.0, 0.55);  // real settings replaces with (1,4) new Settings (3100.0, 0.50);
        KNOWNTARGETING[2][5] = new Settings(3200.0, 0.50);  // real settings replaced with (1,5) new Settings (3100.0, 0.50);
        // ROW 3
        KNOWNTARGETING[3][0] = new Settings(2900.0, 0.50);
        KNOWNTARGETING[3][1] = new Settings(2900.0, 0.50);
        KNOWNTARGETING[3][2] = new Settings(2900.0, 0.50);
        KNOWNTARGETING[3][3] = new Settings(3100.0, 0.47);
        KNOWNTARGETING[3][4] = new Settings(3100.0, 0.47);
        KNOWNTARGETING[3][5] = new Settings(3100.0, 0.47);
        // ROW 4
        KNOWNTARGETING[4][0] = new Settings(3100, 0.49);
        KNOWNTARGETING[4][1] = new Settings(3100, 0.49);
        KNOWNTARGETING[4][2] = new Settings(3100, 0.5);
        KNOWNTARGETING[4][3] = new Settings(3200, 0.5);
        KNOWNTARGETING[4][4] = new Settings(3250, 0.49);
        KNOWNTARGETING[4][5] = new Settings(3300, 0.49);
        // ROW 5
        KNOWNTARGETING[5][0] = new Settings(3200, 0.48);
        KNOWNTARGETING[5][1] = new Settings(3200, 0.48);
        KNOWNTARGETING[5][2] = new Settings(3300, 0.48);
        KNOWNTARGETING[5][3] = new Settings(3350, 0.48);
        KNOWNTARGETING[5][4] = new Settings(3350, 0.48);
        KNOWNTARGETING[5][5] = new Settings(3350, 0.48);

    }

    public final int TILE_UPPER_QUARTILE = 18;
    public final int TILE_LOWER_QUARTILE = 6;
    public double robotInchesX, robotInchesY = 0.0;
    public int robotGridX, robotGridY = 0;
    MultipleTelemetry TELE;
    double cancelOffsetX = 0.0;  // was -40.0
    double cancelOffsetY = 0.0;  // was 7.0
    double unitConversionFactor = 0.95;
    int tileSize = 24; //inches
    public static boolean turretInterpolate = false;

    public Targeting() {
    }

    double cos54 = Math.cos(Math.toRadians(-54));
    double sin54 = Math.sin(Math.toRadians(-54));

    public Settings calculateSettings(double robotX, double robotY, double robotHeading, double robotVelocity, boolean interpolate) {
        Settings recommendedSettings = new Settings(0.0, 0.0);
        if (!redAlliance){
            sin54 = Math.sin(Math.toRadians(54));
        } else {
            sin54 = Math.sin(Math.toRadians(-54));
        }
        // TODO: test these values determined from the fmap
        double rotatedY = (robotX + cancelOffsetX) * sin54 + (robotY + cancelOffsetY) * cos54;
        double rotatedX = (robotX + cancelOffsetX) * cos54 - (robotY + cancelOffsetY) * sin54;

        // Convert robot coordinates to inches
        robotInchesX = rotatedX * unitConversionFactor;
        robotInchesY = rotatedY * unitConversionFactor;

        // Find approximate location in the grid
        int gridX = Math.abs(Math.floorDiv((int) robotInchesX, tileSize) + 1);
        int gridY = Math.abs(Math.floorDiv((int) robotInchesY, tileSize));

        int remX = Math.floorMod((int) robotInchesX, tileSize);
        int remY = Math.floorMod((int) robotInchesY, tileSize);

        //clamp

        //if (redAlliance) {
            robotGridX = Math.max(0, Math.min(gridX, KNOWNTARGETING[0].length - 1));
            robotGridY = Math.max(0, Math.min(gridY, KNOWNTARGETING.length - 1));
        //} else {
//            robotGridX = Math.max(0, Math.min(gridX, KNOWNTARGETING[0].length - 1));
//            robotGridY = Math.max(0, Math.min(gridY, KNOWNTARGETING.length - 1));
        //}

        // Determine if we need to interpolate based on tile position.
        // if near upper or lower quarter or tile interpolate with next tile.
        int x0 = 0;
        int y0 = 0;
        int x1 = 0;
        int y1 = 0;
        interpolate = false;
        if ((remX > TILE_UPPER_QUARTILE) && (remY > TILE_UPPER_QUARTILE) &&
                (robotGridX < 5) && (robotGridY < 5)) {
            // +X, +Y
            interpolate = true;
            x0 = robotGridX;
            x1 = robotGridX + 1;
            y0 = robotGridY;
            y1 = robotGridY + 1;
        } else if ((remX < TILE_LOWER_QUARTILE) && (remY < TILE_LOWER_QUARTILE) &&
                (robotGridX > 0) && (robotGridY > 0)) {
            // -X, -Y
            interpolate = true;
            x0 = robotGridX - 1;
            x1 = robotGridX;
            y0 = robotGridY - 1;
            y1 = robotGridY;
        } else if ((remX > TILE_UPPER_QUARTILE) && (remY < TILE_LOWER_QUARTILE) &&
                (robotGridX < 5) && (robotGridY > 0)) {
            // +X, -Y
            interpolate = true;
            x0 = robotGridX;
            x1 = robotGridX + 1;
            y0 = robotGridY - 1;
            y1 = robotGridY;
        } else if ((remX < TILE_LOWER_QUARTILE) && (remY > TILE_UPPER_QUARTILE) &&
                (robotGridX > 0) && (robotGridY < 5)) {
            // -X, +Y
            interpolate = true;
            x0 = robotGridX - 1;
            x1 = robotGridX;
            y0 = robotGridY;
            y1 = robotGridY + 1;
        } else if ((remX < TILE_LOWER_QUARTILE) && (robotGridX > 0)) {
            // -X, Y
            interpolate = true;
            x0 = robotGridX - 1;
            x1 = robotGridX;
            y0 = robotGridY;
            y1 = robotGridY;
        } else if ((remY < TILE_LOWER_QUARTILE) && (robotGridY > 0)) {
            // X, -Y
            interpolate = true;
            x0 = robotGridX;
            x1 = robotGridX;
            y0 = robotGridY - 1;
            y1 = robotGridY;
        } else if ((remX > TILE_UPPER_QUARTILE) && (robotGridX < 5)) {
            // +X, Y
            interpolate = true;
            x0 = robotGridX;
            x1 = robotGridX + 1;
            y0 = robotGridY;
            y1 = robotGridY;
        } else if ((remY > TILE_UPPER_QUARTILE) && (robotGridY < 5)) {
            // X, +Y
            interpolate = true;
            x0 = robotGridX;
            x1 = robotGridX;
            y0 = robotGridY;
            y1 = robotGridY + 1;
        } else {
            interpolate = false;
        }

        // basic search
        if (true) { //!interpolate) {
            if ((robotGridY < 6) && (robotGridX < 6)) {
                recommendedSettings.flywheelRPM = KNOWNTARGETING[robotGridX][robotGridY].flywheelRPM;
                recommendedSettings.hoodAngle = KNOWNTARGETING[robotGridX][robotGridY].hoodAngle;
            }
            return recommendedSettings;
        } else {

            // bilinear interpolation
            //int x0 = robotGridX;
            //int x1 = Math.min(x0 + 1, KNOWNTARGETING[0].length - 1);
            //int y0 = robotGridY;
            //int y1 = Math.min(y0 + 1, KNOWNTARGETING.length - 1);

//            double x = (robotInchesX - (x0 * tileSize)) / tileSize;
//            double y = (robotInchesY - (y0 * tileSize)) / tileSize;

//            double rpm00 = KNOWNTARGETING[y0][x0].flywheelRPM;
//            double rpm10 = KNOWNTARGETING[y0][x1].flywheelRPM;
//            double rpm01 = KNOWNTARGETING[y1][x0].flywheelRPM;
//            double rpm11 = KNOWNTARGETING[y1][x1].flywheelRPM;
//
//            double angle00 = KNOWNTARGETING[y0][x0].hoodAngle;
//            double angle10 = KNOWNTARGETING[y0][x1].hoodAngle;
//            double angle01 = KNOWNTARGETING[y1][x0].hoodAngle;
//            double angle11 = KNOWNTARGETING[y1][x1].hoodAngle;

//            recommendedSettings.flywheelRPM = (1 - x) * (1 - y) * rpm00 + x * (1 - y) * rpm10 + (1 - x) * y * rpm01 + x * y * rpm11;
//            recommendedSettings.hoodAngle = (1 - x) * (1 - y) * angle00 + x * (1 - y) * angle10 + (1 - x) * y * angle01 + x * y * angle11;
            // Average target tiles
            recommendedSettings.flywheelRPM = (KNOWNTARGETING[x0][y0].flywheelRPM + KNOWNTARGETING[x1][y1].flywheelRPM) / 2.0;
            recommendedSettings.hoodAngle = (KNOWNTARGETING[x0][y0].hoodAngle + KNOWNTARGETING[x1][y1].hoodAngle) / 2.0;
            return recommendedSettings;
        }
    }

    public static class Settings {
        public double flywheelRPM = 0.0;
        public double hoodAngle = 0.0;

        public Settings(double flywheelRPM, double hoodAngle) {
            this.flywheelRPM = flywheelRPM;
            this.hoodAngle = hoodAngle;
        }
    }
}
