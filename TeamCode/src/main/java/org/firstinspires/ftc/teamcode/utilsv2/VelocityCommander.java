package org.firstinspires.ftc.teamcode.utilsv2;

public class VelocityCommander {
    private final double goalH = 20.0; //TODO: Tune
    private final double xVelK = 0; // TODO: Tune
    private final double xAccK = 0; // TODO: Tune
    private final double yVelK = 0; // TODO: Tune
    private final double yAccK = 0; // TODO: Tune

    public VelocityCommander() {

    }

    private double distToRPM (double dist){
        return Math.sqrt(dist*dist + goalH*goalH);
        //TODO: Add regression here using goalH
    }

    public double getVeloStationary (double distance){
        return distToRPM(distance);
    }

    public double getVeloPredictive(double dx, double dy, double xVel, double xAcc, double yVel, double yAcc) {

        double predictedDx = dx - (xVel * xVelK) - (0.5 * xAcc * xAccK); // Negative bc dx = target - robot
        double predictedDy = dy - (yVel * yVelK) - (0.5 * yAcc * yAccK);  // Negative bc dy = target - robot

        double predictedDist = Math.sqrt(dx*dx + dy*dy);

        return distToRPM(predictedDist);
    }


}
