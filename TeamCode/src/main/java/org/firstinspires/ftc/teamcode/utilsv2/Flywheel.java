package org.firstinspires.ftc.teamcode.utilsv2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.LinkedList;

public class Flywheel {
    Robot robot;

    public PIDFCoefficients shooterPIDF1, shooterPIDF2;

    private double velo = 0.0;
    private double velo1 = 0.0;
    private double velo2 = 0.0;

    private double averageVelocity = 0.0;

    double targetVelocity = 0.0;

    boolean steady = false;

    private final LinkedList<Double> velocityHistory = new LinkedList<>();

    public Flywheel(HardwareMap hardwareMap) {

        robot = new Robot(hardwareMap);
        shooterPIDF1 = new PIDFCoefficients(Robot.shooterPIDF_P, Robot.shooterPIDF_I, Robot.shooterPIDF_D, Robot.shooterPIDF_F);
        shooterPIDF2 = new PIDFCoefficients(Robot.shooterPIDF_P, Robot.shooterPIDF_I, Robot.shooterPIDF_D, Robot.shooterPIDF_F);
    }

    public double getVelo() {
        return velo;
    }

    public double getVelo1() {
        return velo1;
    }

    public double getVelo2() {
        return velo2;
    }

    public double getAverageVelocity() {
        return averageVelocity;
    }

    public boolean getSteady() {
        return steady;
    }

    // Set the robot PIDF for the next cycle.
    private double prevF = 0;

    public static double voltagePIDFDifference = 0.8;

    public void setPIDF(double p, double i, double d, double f) {

        shooterPIDF1.p = p;
        shooterPIDF1.i = i;
        shooterPIDF1.d = d;
        shooterPIDF1.f = f;

        shooterPIDF2.p = p;
        shooterPIDF2.i = i;
        shooterPIDF2.d = d;
        shooterPIDF2.f = f;

        if (Math.abs(prevF - f) > voltagePIDFDifference) {

            robot.shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF1);

            robot.shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF2);

            prevF = f;
        }
    }

    // Convert from RPM to Ticks per Second
    private double RPM_to_TPS(double RPM) {
        return (RPM * 28.0) / 60.0;
    }

    // Convert from Ticks per Second to RPM
    private double TPS_to_RPM(double TPS) {
        return (TPS * 60.0) / 28.0;
    }

    private void updateVelocityAverage(double newVelocity) {

        velocityHistory.add(newVelocity);

        int velocityHistorySize = 5;
        if (velocityHistory.size() > velocityHistorySize) {
            velocityHistory.removeFirst();
        }

        double sum = 0.0;

        for (double v : velocityHistory) {
            sum += v;
        }

        averageVelocity = sum / velocityHistory.size();
    }

    public void manageFlywheel(double commandedVelocity) {

        if (Math.abs(targetVelocity - commandedVelocity) > 0.0001) {
            targetVelocity = commandedVelocity;
        }

        robot.shooter1.setVelocity(RPM_to_TPS(targetVelocity));

        robot.shooter2.setVelocity(RPM_to_TPS(targetVelocity));

        velo1 = TPS_to_RPM(robot.shooter1.getVelocity());

        velo2 = TPS_to_RPM(robot.shooter2.getVelocity());

        velo = (velo1 + velo2) / 2.0;

        updateVelocityAverage(velo);

        steady = (Math.abs(commandedVelocity - averageVelocity) < 50);
    }
}