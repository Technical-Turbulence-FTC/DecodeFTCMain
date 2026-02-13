package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Back_Poses {
    public static double rLeaveX = 90, rLeaveY = 80, rLeaveH = 50.1;
    public static double bLeaveX = 90, bLeaveY = -80, bLeaveH = -50;

    public static double rShootX = 95, rShootY = 85, rShootH = 90;
    public static double bShootX = 95, bShootY = -85, bShootH = -90;

    public static double rStackPickupAX = 75, rStackPickupAY = 53, rStackPickupAH = 140;
    public static double bStackPickupAX = 75, bStackPickupAY = -53, bStackPickupAH = -140;

    public static double rStackPickupBX = 50, rStackPickupBY = 78, rStackPickupBH = 140.1;
    public static double bStackPickupBX = 50, bStackPickupBY = -78, bStackPickupBH = -140.1;

    public static Pose2d autoStart = new Pose2d(0, 0, 0); // TODO: find this position


}
