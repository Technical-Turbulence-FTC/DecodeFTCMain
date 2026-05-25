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


    public Turret (Robot rob){
        this.robot = rob;
    }

    public void trackGoal (double dx, double dy, double h) {
        // dx, dy, dz is target - robot
        // h is the raw heading where 0 degrees is positive x in the system of x, y
        while (h > 180) h -= 360;
        while (h < -180) h += 360;

        double fieldRelativeHeading = Math.toDegrees(Math.atan2(dy,dx)); // Angle assuming the robot is at zero degrees, CCW
        double desiredAngle = fieldRelativeHeading - h; // Account for robot rotation
        double angleDelta = desiredAngle - 180; // Subtract 180 as the neutral position is at 180 degrees

        // Shift to -180 --> 180 scale
        while (angleDelta > 180) angleDelta -= 360;
        while (angleDelta < -180) angleDelta += 360;

        double servoTicksFromNeutral = (angleDelta / 360.0) * (2 * servoTicksPer180);
        double servoAngle = neutralPosition + servoTicksFromNeutral;
        servoAngle = Range.clip(servoAngle, turretMin, turretMax);

        robot.turr1.setPosition(servoAngle);
        robot.turr2.setPosition(1.0 - servoAngle);










    }

}
