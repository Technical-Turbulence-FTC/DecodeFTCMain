package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.ShooterVars.kP;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.maxStep;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class Flywheel {
    Robot robot;
    MultipleTelemetry TELE;

    double initPos = 0.0;
    double stamp = 0.0;
    double stamp1 = 0.0;
    double ticker = 0.0;
    double currentPos = 0.0;
    double velo = 0.0;
    double velo1 = 0.0;
    double velo2 = 0.0;
    double velo3 = 0.0;
    double velo4 = 0.0;
    double velo5 = 0.0;
    double targetVelocity = 0.0;
    double powPID = 0.0;
    boolean steady = false;
    public Flywheel () {
        //robot = new Robot(hardwareMap);
    }

    public double getVelo () {
        return velo;
    }

    public boolean getSteady() {
        return steady;
    }

    private double getTimeSeconds ()
    {
        return (double) System.currentTimeMillis()/1000.0;
    }


    public double manageFlywheel(int commandedVelocity, double shooter1CurPos) {
        targetVelocity = commandedVelocity;

        ticker++;
        if (ticker % 2 == 0) {
            velo5 = velo4;
            velo4 = velo3;
            velo3 = velo2;
            velo2 = velo1;

            currentPos = shooter1CurPos / 2048;
            stamp = getTimeSeconds(); //getRuntime();
            velo1 = -60 * ((currentPos - initPos) / (stamp - stamp1));
            initPos = currentPos;
            stamp1 = stamp;

            velo = (velo1 + velo2 + velo3 + velo4 + velo5) / 5;
        }
        // Flywheel control code here
        if (targetVelocity - velo > 500) {
            powPID = 1.0;
        } else if (velo - targetVelocity > 500){
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

        // really should be a running average of the last 5
        steady = (Math.abs(targetVelocity - velo) < 100.0);

        return powPID;
    }

    public void update()
    {
    }
}