package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public class ColorSensorSubsystem {
    private Robot robot;

    // Color detection lists for each sensor
    private List<Double> s1G = new ArrayList<>();
    private List<Double> s2G = new ArrayList<>();
    private List<Double> s3G = new ArrayList<>();
    private List<Double> s1T = new ArrayList<>();
    private List<Double> s2T = new ArrayList<>();
    private List<Double> s3T = new ArrayList<>();
    private List<Boolean> s1 = new ArrayList<>();
    private List<Boolean> s2 = new ArrayList<>();
    private List<Boolean> s3 = new ArrayList<>();

    // Current ball colors
    private boolean green1 = false;
    private boolean green2 = false;
    private boolean green3 = false;

    public ColorSensorSubsystem(Robot robot) {
        this.robot = robot;
    }

    /**
     * Update color sensors and detect ball colors
     * @param runtime Current runtime for timestamps
     */
    public void update(double runtime) {
        double s1D = robot.color1.getDistance(DistanceUnit.MM);
        double s2D = robot.color2.getDistance(DistanceUnit.MM);
        double s3D = robot.color3.getDistance(DistanceUnit.MM);

        // Sensor 1
        if (s1D < 40) {
            double green = robot.color1.getNormalizedColors().green;
            double red = robot.color1.getNormalizedColors().red;
            double blue = robot.color1.getNormalizedColors().blue;

            double gP = green / (green + red + blue);
            s1G.add(gP);

            if (gP >= 0.43) {
                s1.add(true);
            } else {
                s1.add(false);
            }

            s1T.add(runtime);
        }

        // Sensor 2
        if (s2D < 40) {
            double green = robot.color2.getNormalizedColors().green;
            double red = robot.color2.getNormalizedColors().red;
            double blue = robot.color2.getNormalizedColors().blue;

            double gP = green / (green + red + blue);
            s2G.add(gP);

            if (gP >= 0.43) {
                s2.add(true);
            } else {
                s2.add(false);
            }

            s2T.add(runtime);
        }

        // Sensor 3
        if (s3D < 30) {
            double green = robot.color3.getNormalizedColors().green;
            double red = robot.color3.getNormalizedColors().red;
            double blue = robot.color3.getNormalizedColors().blue;

            double gP = green / (green + red + blue);
            s3G.add(gP);

            if (gP >= 0.43) {
                s3.add(true);
            } else {
                s3.add(false);
            }

            s3T.add(runtime);
        }

        // Update green flags
        if (!s1.isEmpty()) {
            green1 = checkGreen(s1, s1T);
        }
        if (!s2.isEmpty()) {
            green2 = checkGreen(s2, s2T);
        }
        if (!s3.isEmpty()) {
            green3 = checkGreen(s3, s3T);
        }
    }

    /**
     * Check if majority of recent readings indicate green
     */
    private boolean checkGreen(List<Boolean> s, List<Double> sT) {
        if (s.isEmpty()) return false;

        double lastTime = sT.get(sT.size() - 1);
        int countTrue = 0;
        int countWindow = 0;

        for (int i = 0; i < s.size(); i++) {
            if (lastTime - sT.get(i) <= 3.0) {
                countWindow++;
                if (s.get(i)) {
                    countTrue++;
                }
            }
        }

        if (countWindow == 0) return false;
        return countTrue > countWindow / 2.0;
    }

    /**
     * Check if ball is present in slot (based on recent sensor readings)
     */
    public boolean ballIn(int slot, double runtime) {
        switch (slot) {
            case 1:
                if (!s1T.isEmpty()) {
                    return !(s1T.get(s1T.size() - 1) < runtime - 3);
                }
                break;
            case 2:
                if (!s2T.isEmpty()) {
                    return !(s2T.get(s2T.size() - 1) < runtime - 3);
                }
                break;
            case 3:
                if (!s3T.isEmpty()) {
                    return !(s3T.get(s3T.size() - 1) < runtime - 3);
                }
                break;
        }
        return true;
    }

    /**
     * Get ball color for specific slot
     */
    public boolean getBallColor(int slot) {
        switch (slot) {
            case 1:
                return green1;
            case 2:
                return green2;
            case 3:
                return green3;
        }
        return false;
    }

    public boolean isGreen1() {
        return green1;
    }

    public boolean isGreen2() {
        return green2;
    }

    public boolean isGreen3() {
        return green3;
    }
}
