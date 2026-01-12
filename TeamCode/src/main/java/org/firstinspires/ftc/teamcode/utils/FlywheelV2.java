package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FlywheelV2 {
    public static double kP = 0.001;           // small proportional gain (tune this)
    public static double maxStep = 0.06;         // prevents sudden jumps
    double initPos1 = 0.0;
    double initPos2 = 0.0;
    double stamp = 0.0;
    double stamp1 = 0.0;
    double ticker = 0.0;
    double currentPos1 = 0.0;
    double currentPos2 = 0.0;
    double velo = 0.0;
    double velo1 = 0.0;
    double velo1a = 0.0;
    double velo1b = 0.0;
    double velo2 = 0.0;
    double velo3 = 0.0;
    double velo4 = 0.0;
    double velo5 = 0.0;
    double targetVelocity = 0.0;
    double powPID = 0.0;
    boolean steady = false;

    public FlywheelV2() {
        //robot = new Robot(hardwareMap);
    }

    public double getVelo(double shooter1CurPos, double shooter2CurPos) {
        ticker++;
        if (ticker % 2 == 0) {
            velo5 = velo4;
            velo4 = velo3;
            velo3 = velo2;
            velo2 = velo1;

            currentPos1 = shooter1CurPos / 28;
            currentPos2 = shooter2CurPos / 28;
            stamp = getTimeSeconds(); //getRuntime();
            velo1a = 60 * ((currentPos1 - initPos1) / (stamp - stamp1));
            velo1b = 60 * ((currentPos2 - initPos2) / (stamp - stamp1));
            initPos1 = currentPos1;
            initPos2 = currentPos2;
            stamp1 = stamp;

            if (velo1a < 200){
                velo1 = velo1b;
            } else if (velo1b < 200){
                velo1 = velo1a;
            } else {
                velo1 = (velo1a + velo1b) / 2;
            }
        }
        return ((velo1 + velo2 + velo3 + velo4 + velo5) / 5);
    }

    public double getVelo1() { return (velo1a + velo2 + velo3 + velo4 + velo5) / 5; }

    public double getVelo2() { return (velo1b + velo2 + velo3 + velo4 + velo5) / 5; }

    public boolean getSteady() {
        return steady;
    }

    private double getTimeSeconds() {
        return (double) System.currentTimeMillis() / 1000.0;
    }

    public double manageFlywheel(int commandedVelocity, double shooter1CurPos, double shooter2CurPos) {
        targetVelocity = commandedVelocity;
        velo = getVelo(shooter1CurPos, shooter2CurPos);
        // Flywheel PID code here
        if (targetVelocity - velo > 500) {
            powPID = 1.0;
        } else if (velo - targetVelocity > 500) {
            powPID = 0.0;
        } else {
            double feed = Math.log((668.39 / (targetVelocity + 591.96)) - 0.116) / -4.18;

            // --- PROPORTIONAL CORRECTION ---
            double error = targetVelocity - velo;
            double correction = kP * error;

            // limit how fast power changes (prevents oscillation)
            correction = Math.max(-maxStep, Math.min(maxStep, correction));

            // --- FINAL MOTOR POWER ---
            powPID = feed + correction;

            // clamp to allowed range
            powPID = Math.max(0, Math.min(1, powPID));
        }

        steady = (Math.abs(targetVelocity - velo) < 100.0);

        return powPID;
    }

    public void update() {
    }
}