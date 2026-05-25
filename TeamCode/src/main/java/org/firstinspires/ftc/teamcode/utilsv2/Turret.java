package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
public class Turret {
    Robot robot;

    private final double servoTicksPer180 = 0.6; // TODO: Tune
    private final double neutralPosition = 0.3; //TODO: Tune
    private final double turretMin = 0.04; //TODO: Tune
    private final double turretMax = 0.94; //TODO: Tune

    private final double hVelK = 0.12; // TODO: Tune
    private final double hAccK = 0.02; // TODO: Tune

    private final double xVelK = 0.10; // TODO: Tune
    private final double xAccK = 0.02; // TODO: Tune

    private final double yVelK = 0.10; // TODO: Tune
    private final double yAccK = 0.02; // TODO: Tune


    public Turret(Robot rob) {
        this.robot = rob;
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public void trackGoal(double dx, double dy, double h, double hVel, double hAcc, double xVel, double xAcc, double yVel, double yAcc) {
        // dx, dy, dz is target - robot
        // h is the raw heading where 0 degrees is positive x in the system of x, y

        double predictedDx = dx - (xVel * xVelK) - (0.5 * xAcc * xAccK); // Negative bc dx = target - robot
        double predictedDy = dy - (yVel * yVelK) - (0.5 * yAcc * yAccK);  // Negative bc dy = target - robot
        double predictedH = h + (hVel * hVelK) + (0.5 * hAcc * hAccK); // Positive bc h = robot heading

        predictedH = wrapAngle(predictedH);

        double fieldRelativeHeading = Math.atan2(predictedDy, predictedDx);

        double desiredAngle = fieldRelativeHeading - predictedH;
        double angleDelta = desiredAngle - Math.PI;
        angleDelta = wrapAngle(angleDelta);

        double servoTicksFromNeutral = (angleDelta / (2.0 * Math.PI)) * (2.0 * servoTicksPer180);

        double servoAngle = neutralPosition + servoTicksFromNeutral;

        servoAngle = Range.clip(servoAngle, turretMin, turretMax);

        robot.turr1.setPosition(servoAngle);
        robot.turr2.setPosition(1.0 - servoAngle);
    }
}