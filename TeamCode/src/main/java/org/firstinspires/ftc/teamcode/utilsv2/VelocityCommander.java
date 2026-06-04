package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VelocityCommander {
    public static double xVelK = 0.05; // TODO: Tune
    public static double xAccK = 0.025; // TODO: Tune
    public static double yVelK = 0.05; // TODO: Tune
    public static double yAccK = 0.025; // TODO: Tune
    private double hoodPos = 0.88;
    private double transferPow = -1;
    private double velo = 0;

    public VelocityCommander() {}

    private final double veloA = -2.703087757*Math.pow(10, -14);
    private final double veloB = 2.904756341*Math.pow(10, -11);
    private final double veloC = -1.381814293*Math.pow(10, -8);
    private final double veloD = 0.000003829224585;
    private final double veloE = -0.000684090204;
    private final double veloF = 0.0822754689;
    private final double veloG = -6.743119277;
    private final double veloH = 371.7359504;
    private final double veloI = -13189.70958;
    private final double veloJ = 272005.7124;
    private final double veloK = -2474581.713;
    private double distToRPM (double dist){
        if (dist < 49) {
            velo = 2000;
        } else if (dist > 165){
            velo = 3760;
        } else {
            velo = veloA*Math.pow(dist, 10) +
                    veloB*Math.pow(dist, 9) +
                    veloC*Math.pow(dist, 8) +
                    veloD*Math.pow(dist, 7) +
                    veloE*Math.pow(dist, 6) +
                    veloF*Math.pow(dist, 5) +
                    veloG*Math.pow(dist, 4) +
                    veloH*Math.pow(dist, 3) +
                    veloI*Math.pow(dist, 2) +
                    veloJ*Math.pow(dist, 1) +
                    veloK;
            velo = Math.max(2000, Math.min(3760, velo));
        }
        return velo;
    }

    private final double hoodA = -4.3276177*Math.pow(10, -13);
    private final double hoodB = 2.68062979*Math.pow(10, -10);
    private final double hoodC = -7.12859632*Math.pow(10, -8);
    private final double hoodD = 0.0000106010785;
    private final double hoodE = -0.000960693973;
    private final double hoodF = 0.0540375808;
    private final double hoodG = -1.82724027;
    private final double hoodH = 33.4797545;
    private final double hoodI = -246.888632;
    private void distToHood (double dist){
        if (dist > 112){
            hoodPos = 0.35;
        } else if (dist < 49){
            hoodPos = 0.88;
        } else {
            hoodPos = hoodA*Math.pow(dist, 8) +
                    hoodB*Math.pow(dist, 7) +
                    hoodC*Math.pow(dist, 6) +
                    hoodD*Math.pow(dist, 5) +
                    hoodE*Math.pow(dist, 4) +
                    hoodF*Math.pow(dist, 3) +
                    hoodG*Math.pow(dist, 2) +
                    hoodH*Math.pow(dist, 1) +
                    hoodI;

            hoodPos = Math.max(0.35, Math.min(0.88, hoodPos));
        }
    }
    public double getHoodPredicted(){
        return hoodPos;
    }

    private void distToTransferPow(double dist, double voltage){
        if (dist < 118){
            transferPow = -1;
        } else if (dist < 125){
            transferPow = -0.7;
        } else {
            transferPow = -0.5;
        }

//        transferPow = Math.max(-0.5, Math.min(-1, transferPow * (14/voltage)));
    }
    public double getTransferPow(){return transferPow;}

    // 27
    public double getVeloStationary (double distance){
        return distToRPM(distance);
    }

    double predictedDist = 0;
    public void getVeloPredictive(double dx, double dy, double xVel, double xAcc, double yVel, double yAcc, double voltage) {

        double predictedDx = dx - (xVel * xVelK) - (0.5 * xAcc * xAccK); // Negative bc dx = target - robot
        double predictedDy = dy - (yVel * yVelK) - (0.5 * yAcc * yAccK);  // Negative bc dy = target - robot

        double goalHeight = 28;
        predictedDist = Math.sqrt(predictedDx*predictedDx + predictedDy*predictedDy + goalHeight * goalHeight);

        distToHood(predictedDist);
        distToTransferPow(predictedDist, voltage);
        distToRPM(predictedDist);
    }

    public double getPredictedRPM(){return velo;}

    public double getDistance(){return predictedDist;}
}
