package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Poses_V2 {

    public static double goalHeight = 42; //in inches

    public static double turretHeight = 12;

    public static double relativeGoalHeight = goalHeight - turretHeight;

    public static Pose2d goalPose = new Pose2d(-10, 0, 0);

    public static double rx1 = 20, ry1 = 0, rh1 = 0;
    public static double rx2a = 55, ry2a = 39, rh2a = Math.toRadians(140), rt2a  = Math.toRadians(Math.PI/2);
    public static double rx2b = 33, ry2b = 61, rh2b = Math.toRadians(140), rt2b  = Math.toRadians(Math.PI/2);

    public static double rx2c = 34, ry2c = 50, rh2c = Math.toRadians(140), rt2c  = Math.toRadians(Math.PI/2);

    public static double rXGateA = 27, rYGateA = 56, rHGateA = Math.toRadians(160);

    public static double rXGateB = 40, rYGateB = 43, rHGateB = Math.toRadians(159);

    public static double rXGate = 30, rYGate = 63, rHGate = Math.toRadians(179);

    public static double rx3a = 55, ry3a = 39, rh3a = Math.toRadians(140);
    public static double rx3b = 33, ry3b = 61, rh3b = Math.toRadians(140);

    public static double rx4a = 72, ry4a = 55, rh4a = Math.toRadians(140);
    public static double rx4b = 48, ry4b = 79, rh4b = Math.toRadians(140);

    public static double bx1 = 20, by1 = 0, bh1 = 0;
    public static double bx2a = 45, by2a = -18, bh2a = Math.toRadians(-140), bt2a  = Math.toRadians(140);
    public static double bx2b = 25, by2b = -38, bh2b = Math.toRadians(-140), bt2b  = Math.toRadians(140);
    public static double bx2c = 34, by2c = -50, bh2c = Math.toRadians(-140), bt2c  = Math.toRadians(140);

    public static double rShootX = 40, rShootY = 7, rShootH = Math.toRadians(140);

    public static double bShootX = 20, bShootY = 30, bShootH = Math.toRadians(140);

    public static double bXGateA = 33, bYGateA = 61, bHGateA = Math.toRadians(165);
    public static double bXGateB = 33, bYGateB = 61, bHGateB = Math.toRadians(165);

    public static double bXGate = 25, bYGate = 69, bHGate = Math.toRadians(165);


    public static double bx3a = 55, by3a = -43, bh3a = Math.toRadians(-140);
    public static double bx3b = 37, by3b = -61, bh3b = Math.toRadians(-140);

    public static double bx4a = 72, by4a = -55, bh4a = Math.toRadians(-140);
    public static double bx4b = 48, by4b = -79, bh4b = Math.toRadians(-140);
    public static double rfx1 = 10, rfy1 = 0, rfh1 = 0; //TODO: test this

    public static double rShoot1Tangent = Math.toRadians(0);
    public static double bShoot1Tangent = Math.toRadians(0);



    public static Pose2d teleStart = new Pose2d(0, 0, 0);

}
