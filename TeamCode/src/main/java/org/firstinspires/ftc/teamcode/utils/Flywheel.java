package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.ShooterVars.kP;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.maxStep;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    double targetVelocity = 0.0;
    double powPID = 0.0;
    boolean steady = false;
    public Flywheel (HardwareMap hardwareMap) {
        robot = new Robot(hardwareMap);
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
        robot.shooterPIDF.p = p;
        robot.shooterPIDF.i = i;
        robot.shooterPIDF.d = d;
        robot.shooterPIDF.f = f;
    }
    private double getTimeSeconds ()
    {
        return (double) System.currentTimeMillis()/1000.0;
    }

    // Convert from RPM to Ticks per Second
    private double RPM_to_TPS (double RPM) { return (RPM*28.0)/60.0;}

    // Convert from Ticks per Second to RPM
    private double TPS_to_RPM (double TPS) { return (TPS*60.0)/28.0;}

    public double manageFlywheel(double commandedVelocity) {
        targetVelocity = commandedVelocity;

        // Turn PIDF for Target Velocities
        //robot.shooterPIDF.p = P;
        //robot.shooterPIDF.i = I;
        //robot.shooterPIDF.d = D;
        //robot.shooterPIDF.f = F;
        robot.shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, robot.shooterPIDF);
        robot.shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, robot.shooterPIDF);
        robot.shooter1.setVelocity(RPM_to_TPS(targetVelocity));
        robot.shooter2.setVelocity(RPM_to_TPS(targetVelocity));

        // Record Current Velocity
        velo1 = TPS_to_RPM(robot.shooter1.getVelocity());
        velo2 = TPS_to_RPM(robot.shooter1.getVelocity()); // Possible error: should it be shooter2 not shooter1?
        velo = Math.max(velo1,velo2);

        // really should be a running average of the last 5
        steady = (Math.abs(targetVelocity - velo) < 200.0);

        return powPID;
    }

    public void update()
    {
    }
}
