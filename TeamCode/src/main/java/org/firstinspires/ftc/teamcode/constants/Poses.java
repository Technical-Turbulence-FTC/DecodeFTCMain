package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Poses {

    public static double goalHeight = 42; //in inches

    public static double turretHeight = 12;

    public static double relativeGoalHeight = goalHeight - turretHeight;

    public static Pose2d goalPose = new Pose2d(-15, 0, 0);

    public static double rx1 = 40, ry1 = -7, rh1 = 0;
    public static double rx2a = 41, ry2a = 18, rh2a = Math.toRadians(140);
    public static double rx2b = 23, ry2b = 36, rh2b = Math.toRadians(140);

    public static double rx2c = 34, ry2c = 50, rh2c = Math.toRadians(140);

    public static double rx3a = 55, ry3a = 39, rh3a = Math.toRadians(140);
    public static double rx3b = 33, ry3b = 61, rh3b = Math.toRadians(140);

    public static double rx4a = 72, ry4a = 55, rh4a = Math.toRadians(140);
    public static double rx4b = 48, ry4b = 79, rh4b = Math.toRadians(140);

    public static double bx1 = 40, by1 = 7, bh1 = 0;
    public static double bx2a = 45, by2a = -18, bh2a = Math.toRadians(-140);
    public static double bx2b = 25, by2b = -38, bh2b = Math.toRadians(-140);
    public static double bx2c = 34, by2c = -50, bh2c = Math.toRadians(-140);

    public static double bx3a = 55, by3a = -43, bh3a = Math.toRadians(-140);
    public static double bx3b = 37, by3b = -61, bh3b = Math.toRadians(-140);

    public static double bx4a = 72, by4a = -55, bh4a = Math.toRadians(-140);
    public static double bx4b = 48, by4b = -79, bh4b = Math.toRadians(-140);
    public static double rfx1 = 10, rfy1 = 0, rfh1 = 0; //TODO: test this

    public static Pose2d teleStart = new Pose2d(rx1, ry1, rh1);

}
