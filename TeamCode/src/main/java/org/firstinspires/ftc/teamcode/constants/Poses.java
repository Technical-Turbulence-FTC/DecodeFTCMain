package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Poses {

    public static double goalHeight = 42; //in inches

    public static double turretHeight = 12;

    public static double relativeGoalHeight = goalHeight - turretHeight;

    public static Pose2d goalPose = new Pose2d(-10, 0, 0);

    public static double rx1 = 20, ry1 = 0.5, rh1 = Math.toRadians(0.1);
    public static double rx2a = 41, ry2a = 18, rh2a = Math.toRadians(140);
    public static double rx2b = 23, ry2b = 36, rh2b = Math.toRadians(140.1);

    public static double rx2c = 34, ry2c = 50, rh2c = Math.toRadians(140);

    public static double rx3a = 55, ry3a = 39, rh3a = Math.toRadians(140);
    public static double rx3b = 38, ry3b = 56, rh3b = Math.toRadians(140.1);

    public static double rx4a = 72, ry4a = 50, rh4a = Math.toRadians(145);
    public static double rx4b = 42, ry4b = 80, rh4b = Math.toRadians(145.1);

    public static double bx1 = 40, by1 = 7, bh1 = 0;
    public static double bx2a = 45, by2a = -18, bh2a = Math.toRadians(-140);
    public static double bx2b = 25, by2b = -38, bh2b = Math.toRadians(-140);
    public static double bx2c = 34, by2c = -50, bh2c = Math.toRadians(-140);

    public static double bx3a = 55, by3a = -43, bh3a = Math.toRadians(-140);
    public static double bx3b = 37, by3b = -61, bh3b = Math.toRadians(-140);

    public static double bx4a = 72, by4a = -55, bh4a = Math.toRadians(-140);
    public static double bx4b = 48, by4b = -79, bh4b = Math.toRadians(-140);
    public static double rfx1 = 10, rfy1 = 0, rfh1 = 0; //TODO: test this

    public static double rShootX = 40, rShootY = -7, rShootH = Math.toRadians(50);
    public static double rxPrep = 45, ryPrep = 10, rhPrep = Math.toRadians(50);

    public static double bShootX = 20, bShootY = 30, bShootH = Math.toRadians(140);
    public static double bxPrep = 50, byPrep = -10, bhPrep = Math.toRadians(140);


    public static Pose2d teleStart = new Pose2d(0, 0, 0);

}
