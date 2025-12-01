package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterVars {
    public static double turret_GearRatio = 0.9974;

    public static double turret_Range = 355;

    public static int velTolerance = 300;

    public static int initTolerance = 1000;

    public static int maxVel = 4500;

    public static double waitTransfer = 0.25;
    public static double kP = 0.001;           // small proportional gain (tune this)
    public static double maxStep = 0.06;         // prevents sudden jumps
}
