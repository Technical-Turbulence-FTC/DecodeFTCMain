package org.firstinspires.ftc.teamcode.utils;

public class HoodSubsystem {
    private Robot robot;
    private boolean autoHood = true;
    private double hoodOffset = 0.0;
    private double autoHoodOffset = 0.0;
    public static double hoodDefaultPos = 0.5;

    public HoodSubsystem(Robot robot) {
        this.robot = robot;
    }

    /**
     * Update hood position based on distance to goal
     * @param distanceToGoal Distance to goal in inches
     */
    public void update(double distanceToGoal) {
        if (autoHood) {
            robot.hood.setPosition(hoodAnglePrediction(distanceToGoal) + autoHoodOffset);
        } else {
            robot.hood.setPosition(hoodDefaultPos + hoodOffset);
        }
    }

    /**
     * Calculate hood angle based on distance
     */
    public double hoodAnglePrediction(double x) {
        if (x < 34) {
            double L = 1.04471;
            double U = 0.711929;
            double Q = 120.02263;
            double B = 0.780982;
            double M = 20.61191;
            double v = 10.40506;

            double inner = 1 + Q * Math.exp(-B * (x - M));
            return L + (U - L) / Math.pow(inner, 1.0 / v);
        } else {
            return 1.94372 * Math.exp(-0.0528731 * x) + 0.39;
        }
    }

    /**
     * Adjust hood offset
     */
    public void adjustOffset(double delta) {
        hoodOffset += delta;
    }

    /**
     * Adjust auto hood offset
     */
    public void adjustAutoOffset(double delta) {
        autoHoodOffset += delta;
    }

    /**
     * Set auto hood mode
     */
    public void setAutoMode(boolean auto) {
        this.autoHood = auto;
        if (!auto) {
            hoodOffset = 0;
        }
    }

    public boolean isAutoMode() {
        return autoHood;
    }

    public double getHoodOffset() {
        return hoodOffset;
    }

    public double getAutoHoodOffset() {
        return autoHoodOffset;
    }

    public double getCurrentPosition() {
        return robot.hood.getPosition();
    }
}
