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
    double stamp2 = 0.0;
    double currentPos = 0.0;
    boolean prevSteady = false;
    double velo = 0.0;
    double prevVelo = 0.0;
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
        targetVelocity = (double) commandedVelocity;

        ticker++;
        if (ticker % 8 == 0) {
            currentPos = shooter1CurPos / 2048;
            stamp = getTimeSeconds(); //getRuntime();
            velo = -60 * ((currentPos - initPos) / (stamp - stamp1));
            initPos = currentPos;
            stamp1 = stamp;
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
        if ((Math.abs(targetVelocity - velo) < 150.0) && (Math.abs(targetVelocity - prevVelo) < 150.0))
        {
            steady = true;
        }
        else
        {
            steady = false;
        }

        return powPID;
    }

    public void update()
    {
    };
};