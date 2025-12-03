package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Poses {

    public static double goalHeight = 42; //in inches

    public static double turretHeight = 12;

    public static double relativeGoalHeight = goalHeight - turretHeight;

    public static double x1 = 50, y1 = 0, h1 = 0;

    public static double x2a = 45, y2a = 5, h2a = Math.toRadians(140);

    public static double x2b = 31, y2b = 32, h2b = Math.toRadians(140);

    public static double x3a = 58, y3a = 42, h3a = Math.toRadians(140);

    public static double x3b = 34, y3b = 58, h3b = Math.toRadians(140);

    public static Pose2d teleStart = new Pose2d(x1, -10, 0);

}
