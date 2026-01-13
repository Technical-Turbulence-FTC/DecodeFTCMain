package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Poses {

    public static double goalHeight = 42; //in inches

    public static double turretHeight = 12;

    public static double relativeGoalHeight = goalHeight - turretHeight;

    public static Pose2d goalPose = new Pose2d(-15, 0, 0);

    public static double rx1 = 45, ry1 = -7, rh1 = 0;

    public static double rx2a = 45, ry2a = 5, rh2a = Math.toRadians(140);

    public static double rx2b = 31, ry2b = 32, rh2b = Math.toRadians(140);

    public static double rx3a = 58, ry3a = 42, rh3a = Math.toRadians(140);

    public static double rx3b = 34, ry3b = 58, rh3b = Math.toRadians(140);

    public static double bx1 = 45, by1 = 6, bh1 = 0;

    public static double bx2a = 53, by2a = -7, bh2a = Math.toRadians(-140);

    public static double bx2b = 23, by2b = -39, bh2b = Math.toRadians(-140);

    public static double bx3a = 56, by3a = -34, bh3a = Math.toRadians(-140);

    public static double bx3b = 34, by3b = -58, bh3b = Math.toRadians(-140);

    public static Pose2d teleStart = new Pose2d(rx1, ry1, rh1);

}
