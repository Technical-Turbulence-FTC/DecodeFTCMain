package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class Poses_LT_Indexed {

    // ================= FIELD / GOAL =================
    public static double goalHeight = 42; // inches
    public static double turretHeight = 12;
    public static double relativeGoalHeight = goalHeight - turretHeight;

    public static Pose2d goalPose = new Pose2d(-10, 0, 0);
    public static Pose2d teleStart = new Pose2d(0, 0, 0);

    // =================================================
    // ================= RED ALLIANCE ==================
    // =================================================

    // -------- FIRST SHOT --------
    public static double rx1 = 20, ry1 = 0, rh1 = 0;

    // -------- PICKUP 1 --------
    public static double rx2a = 55, ry2a = 39, rh2a = Math.toRadians(140), rt2a = Math.toRadians(Math.PI / 2);
    public static double rx2b = 33, ry2b = 61, rh2b = Math.toRadians(140), rt2b = Math.toRadians(Math.PI / 2);
    public static double rx2c = 34, ry2c = 50, rh2c = Math.toRadians(140), rt2c = Math.toRadians(Math.PI / 2);

    // -------- OPEN GATE --------
    public static double rXGate = 30, rYGate = 63, rHGate = Math.toRadians(179);

    // -------- PICKUP 2 --------
    public static double rx3 = 0, ry3 = 0, rh3 = 0, rt3 = 0;

    public static double rx3a = 55, ry3a = 39, rh3a = Math.toRadians(140);
    public static double rx3b = 33, ry3b = 61, rh3b = Math.toRadians(140);

    // -------- PICKUP 3 --------
    public static double rx4a = 72, ry4a = 55, rh4a = Math.toRadians(140);
    public static double rx4b = 48, ry4b = 79, rh4b = Math.toRadians(140);

    public static double rx5a = 0, ry5a = 0, rh5a = 0;
    public static double rx5b = 0, ry5b = 0, rh5b = 0;

    // -------- SHOOT --------
    public static double rShootX = 40, rShootY = 7, rShootH = Math.toRadians(140);
    public static double rShoot1Tangent = Math.toRadians(0);

    // -------- PARK --------
    public static double rXPark = 0, rYPark = 0, rHPark = 0;

    // =================================================
    // ================= BLUE ALLIANCE =================
    // =================================================

    // -------- FIRST SHOT --------
    public static double bx1 = 20, by1 = 0, bh1 = 0;

    // -------- PICKUP 1 --------
    public static double bx2a = 45, by2a = -18, bh2a = Math.toRadians(-140), bt2a = Math.toRadians(140);
    public static double bx2b = 25, by2b = -38, bh2b = Math.toRadians(-140), bt2b = Math.toRadians(140);
    public static double bx2c = 34, by2c = -50, bh2c = Math.toRadians(-140), bt2c = Math.toRadians(140);

    // -------- OPEN GATE --------
    public static double bXGate = 25, bYGate = 69, bHGate = Math.toRadians(165);

    // -------- PICKUP 2 --------
    public static double bx3 = 0, by3 = 0, bh3 = 0, bt3 = 0;

    public static double bx3a = 55, by3a = -43, bh3a = Math.toRadians(-140);
    public static double bx3b = 37, by3b = -61, bh3b = Math.toRadians(-140);

    // -------- PICKUP 3 --------
    public static double bx4a = 72, by4a = -55, bh4a = Math.toRadians(-140);
    public static double bx4b = 48, by4b = -79, bh4b = Math.toRadians(-140);

    public static double bx5a = 0, by5a = 0, bh5a = 0;
    public static double bx5b = 0, by5b = 0, bh5b = 0;

    // -------- SHOOT --------
    public static double bShootX = 20, bShootY = 30, bShootH = Math.toRadians(140);
    public static double bShoot1Tangent = Math.toRadians(0);

    // -------- PARK --------
    public static double bXPark = 0, bYPark = 0, bHPark = 0;
}