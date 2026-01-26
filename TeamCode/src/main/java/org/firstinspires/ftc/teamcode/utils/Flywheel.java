package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Flywheel {
    Robot robot;
    public PIDFCoefficients shooterPIDF1, shooterPIDF2;
    double velo = 0.0;
    public double velo1 = 0.0;
    public double velo2 = 0.0;
    double targetVelocity = 0.0;
    double powPID = 0.0;
    boolean steady = false;
    public Flywheel (HardwareMap hardwareMap) {
        robot = new Robot(hardwareMap);
        shooterPIDF1 = new PIDFCoefficients
                (robot.shooterPIDF_P, robot.shooterPIDF_I, robot.shooterPIDF_D, robot.shooterPIDF_F);
        shooterPIDF2 = new PIDFCoefficients
                (robot.shooterPIDF_P, robot.shooterPIDF_I, robot.shooterPIDF_D, robot.shooterPIDF_F);
    }

    public double getVelo () {
        return velo;
    }
    public double getVelo1 () {
        return velo1;
    }
    public double getVelo2 () {
        return velo2;
    }

    public boolean getSteady() {
        return steady;
    }

    // Set the robot PIDF for the next cycle.
    public void setPIDF(double p, double i, double d, double f) {
        shooterPIDF1.p = p;
        shooterPIDF1.i = i;
        shooterPIDF1.d = d;
        shooterPIDF1.f = f;
        shooterPIDF2.p = p;
        shooterPIDF2.i = i;
        shooterPIDF2.d = d;
        shooterPIDF2.f = f;
    }

    // Convert from RPM to Ticks per Second
    private double RPM_to_TPS (double RPM) { return (RPM*28.0)/60.0;}

    // Convert from Ticks per Second to RPM
    private double TPS_to_RPM (double TPS) { return (TPS*60.0)/28.0;}

    public double manageFlywheel(double commandedVelocity) {
        targetVelocity = commandedVelocity;

        // Add code here to set PIDF based on desired RPM

        robot.shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF1);
        robot.shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF2);
        robot.shooter1.setVelocity(RPM_to_TPS(targetVelocity));
        robot.shooter2.setVelocity(RPM_to_TPS(targetVelocity));

        // Record Current Velocity
        velo1 = TPS_to_RPM(robot.shooter1.getVelocity());
        velo2 = TPS_to_RPM(robot.shooter2.getVelocity());
        velo = Math.max(velo1,velo2);

        // really should be a running average of the last 5
        steady = (Math.abs(targetVelocity - velo) < 200.0);

        return powPID;
    }

    public void update()
    {
    }
}
