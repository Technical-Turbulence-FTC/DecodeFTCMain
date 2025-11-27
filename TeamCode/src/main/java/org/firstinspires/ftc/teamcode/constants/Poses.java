package org.firstinspires.ftc.teamcode.constants;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Poses {

    public static double goalHeight = 42; //in inches

    public static double turretHeight = 12;

    public static double relativeGoalHeight = goalHeight - turretHeight;

    public static double x1 = 50, y1 = 0, h1 = 0;

    public static double x2 = 31, y2 = 32, h2 = Math.toRadians(140);

    public static double x2_b = 58, y2_b = 42, h2_b = Math.toRadians(140);


    public static double x3 = 34, y3 = 58, h3 = Math.toRadians(140);

    public static Pose2d teleStart = new Pose2d(x1,-10,0);



}
