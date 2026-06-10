package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VelocityCommander {
    public static double xVelK = 0.05; // TODO: Tune
    public static double xAccK = 0.025; // TODO: Tune
    public static double yVelK = 0.05; // TODO: Tune
    public static double yAccK = 0.025; // TODO: Tune
    public static boolean lockFront = false;
    public static boolean lockBack = false;
    public static int farBound = 138;
    public static int closeBound = 112;
    public static double errorHoodAdjustment = 0.0005;
    private double hoodPos = 0.88;
    private double transferPow = -1;
    private int velo = 0;

    public VelocityCommander() {}

    final double veloA = -0.00000133612;
    final double veloB = 0.000542733;
    final double veloC = -0.0739531;
    final double veloD = 5.16759;
    final double veloE = 62.45781;
    final double veloF = -0.0000833333;
    final double veloG = 0.0377857;
    final double veloH = -4.55067;
    final double veloI = 456.97486;
    private double distToRPM (double dist){
        double currentVelo;
        if (lockFront && dist > closeBound){
            dist = closeBound;
        } else if (lockBack && dist < farBound){
            dist = farBound;
        }
        if (dist < 54) {
            velo = 2000;
        } else if (dist > 174){
            velo = 3700;
        } else {
            currentVelo = veloA*Math.pow(dist, 4) +
                    veloB*Math.pow(dist, 3) +
                    veloC*Math.pow(dist, 2) +
                    veloD*Math.pow(dist, 1) +
                    veloE;
            velo = 10 * Math.round((float) Math.max(200, Math.min(360, currentVelo)));
        }
        return velo;
    }

    final double hoodA = 9.04203*Math.pow(10, -8);
    final double hoodB = -0.0000204165;
    final double hoodC = -0.00252089;
    final double hoodD = 1.06154;
    final double hoodE = -0.002;
    final double hoodF = 0.918;
    private void distToHood (double dist){
        if (dist > 174){
            hoodPos = 0.48;
        } else if (dist < 54){
            hoodPos = 0.88;
        } else {
            hoodPos = hoodA*Math.pow(dist, 3) +
                    hoodB*Math.pow(dist, 2) +
                    hoodC*Math.pow(dist, 1) +
                    hoodD;
        }
        if (lockBack){
            hoodPos-=0.04;
        }

        hoodPos = Math.max(0.48, Math.min(0.88, hoodPos));
    }
    public double getHoodPredicted(){
        return hoodPos;
    }

    private void distToTransferPow(double dist, double voltage){
        if (dist < 140){
            transferPow = -0.85;
        } else {
            transferPow = -0.7;
        }
    }
    public double getTransferPow(){return transferPow;}

    // 27
    public double getVeloStationary (double distance){
        return distToRPM(distance);
    }

    double predictedDist = 0;
    public void getVeloPredictive(double dx, double dy, double xVel, double xAcc, double yVel, double yAcc, double voltage, double velocity) {

        double predictedDx = dx - (xVel * xVelK) - (0.5 * xAcc * xAccK); // Negative bc dx = target - robot
        double predictedDy = dy - (yVel * yVelK) - (0.5 * yAcc * yAccK);  // Negative bc dy = target - robot

        double goalHeight = 28;
        predictedDist = Math.sqrt(predictedDx*predictedDx + predictedDy*predictedDy + goalHeight * goalHeight);

        distToHood(predictedDist);
        distToTransferPow(predictedDist, voltage);
        distToRPM(predictedDist);

//        hoodPos += adjustHood(predictedDist, velocity, velo);
    }

    public double adjustHood(double dist, double currentVelocity, double targetVelocity){
        double error = targetVelocity - currentVelocity;
        if (dist < farBound || error < 0){
            error = 0;
        }
        System.out.println("Error "+ error);
        return error * errorHoodAdjustment;
    }

    public double getPredictedRPM(){return velo;}

    public double getDistance(){return predictedDist;}
}
