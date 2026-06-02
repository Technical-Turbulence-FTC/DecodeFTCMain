package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;


import java.util.List;

@Config
public class Turret {
    Robot robot;

    private final double servoTicksPer180 = 0.58;
    private final double neutralPosition = 0.51;
    private final double turretMin = 0.05;
    private final double turretMax = 0.95;
    private final double hVelK = 0; // TODO: Tune
    private final double xVelK = 0; // TODO: Tune
    private final double xAccK = 0; // TODO: Tune
    private final double yVelK = 0; // TODO: Tune
    private final double yAccK = 0; // TODO: Tune

    private int obeliskID = 0;



    public Turret(Robot rob) {
        this.robot = rob;
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public void trackObelisk(double dx, double dy, double h) {

        double heading = wrapAngle(h);

        double fieldRelativeHeading = Math.atan2(dy, dx);

        double desiredAngle = fieldRelativeHeading - heading;
        double angleDelta = desiredAngle - Math.PI;
        angleDelta = wrapAngle(angleDelta);

        double servoTicksFromNeutral = (angleDelta / (2.0 * Math.PI)) * (2.0 * servoTicksPer180);

        double servoAngle = neutralPosition + servoTicksFromNeutral;

        servoAngle = Range.clip(servoAngle, turretMin, turretMax);

        robot.setTurretPos(servoAngle);


        detectObelisk();

    }

    public int getObeliskID() {
        return obeliskID;
    }

    private int detectObelisk() {
        robot.limelight.pipelineSwitch(1);
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            double prevTx  = -1000;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                double currentTx = fiducial.getTargetXDegrees();
                if (currentTx > prevTx){
                    obeliskID = fiducial.getFiducialId();
                }
            }
        }
        return obeliskID;
    }

    public void manual (double pos) {
        robot.setTurretPos(pos);

    }


    public void trackGoal(double dx, double dy, double h, double hVel, double xVel, double xAcc, double yVel, double yAcc) {
        // dx, dy, dz is target - robot
        // h is the raw heading where 0 degrees is positive x in the system of x, y

        double predictedDx = dx - (xVel * xVelK) - (0.5 * xAcc * xAccK); // Negative bc dx = target - robot
        double predictedDy = dy - (yVel * yVelK) - (0.5 * yAcc * yAccK);  // Negative bc dy = target - robot
        double predictedH = h + (hVel * hVelK); // Positive bc h = robot heading

        predictedH = wrapAngle(predictedH);

        double fieldRelativeHeading = Math.atan2(predictedDy, predictedDx);

        double angleDelta = fieldRelativeHeading - predictedH;
        angleDelta = wrapAngle(angleDelta);

        double servoTicksFromNeutral = (angleDelta / (2.0 * Math.PI)) * (2.0 * servoTicksPer180);

        double servoAngle = neutralPosition + servoTicksFromNeutral;

        servoAngle = Range.clip(servoAngle, turretMin, turretMax);

        robot.setTurretPos(servoAngle);
    }
}