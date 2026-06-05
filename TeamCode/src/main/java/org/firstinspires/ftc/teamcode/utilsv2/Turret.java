package org.firstinspires.ftc.teamcode.utilsv2;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.constants.Color;
import org.firstinspires.ftc.teamcode.teleop.TeleopV3;

import java.util.List;

@Config
public class Turret {
    Robot robot;

    private final double servoTicksPer180 = 0.58;
    private final double neutralPosition = 0.51;
    private final double turretMin = 0.05;
    private final double turretMax = 0.95;
    public static boolean limelightUsed = true;
    public static double B_PID_P = 0.08, B_PID_I = 0.0, B_PID_D = 0.007;
    Limelight3A webcam;
    LLResult result;
    PIDController bearingPID;
    boolean bearingAligned = false;
    private boolean lockOffset = false;
    public int LL_COAST_TICKS = 60;
    public static double TARGET_POSITION_TOLERANCE = 0.5;
    public static double alphaTX = 0.5;
    private double targetTx = 0;
    private double bearingOffset = 0;
    double tx = 0.0;
    double ty = 0.0;
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

    private void limelightRead() { // only for tracking purposes, not general reads
        result = webcam.getLatestResult();
        tx = 1000;
        if (result != null) {
            if (result.isValid()) {
                tx = result.getTx();
            }
        }
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
        result = robot.limelight.getLatestResult();
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

    private double currentTrackOffset = 0;
    private double llCoast = 0;
    private double servoAngle = 0.51;
    public void trackGoal(double dx, double dy, double h, double hVel, double xVel, double xAcc, double yVel, double yAcc) {
        // dx, dy, dz is target - robot
        // h is the raw heading where 0 degrees is positive x in the system of x, y

        double predictedDx = dx - (xVel * xVelK) - (0.5 * xAcc * xAccK); // Negative bc dx = target - robot
        double predictedDy = dy - (yVel * yVelK) - (0.5 * yAcc * yAccK);  // Negative bc dy = target - robot
        double predictedH = h + (hVel * hVelK); // Positive bc h = robot heading

        predictedH = wrapAngle(predictedH);

        double fieldRelativeHeading = Math.atan2(predictedDy, predictedDx);

        double angleDelta = fieldRelativeHeading - predictedH;
        angleDelta = wrapAngle(angleDelta) / (2.0 * Math.PI);

        double bearingOffset = 0;
        if (limelightUsed && servoAngle > turretMin && servoAngle < turretMax){
            limelightRead();
            if (result.isValid() && tx < 100){
                targetTx = (tx*alphaTX)+(targetTx*(1-alphaTX));
                bearingAligned = Math.abs(targetTx) < TARGET_POSITION_TOLERANCE;
                if (!bearingAligned){
                    bearingOffset = -(bearingPID.calculate(targetTx, 0.0));
                }
            } else {
                targetTx = 0;
                bearingOffset = 0;
            }
            currentTrackOffset += bearingOffset;
            llCoast = LL_COAST_TICKS;
        } else {
            if (llCoast <= 0){
                currentTrackOffset = 0;
            } else {
                llCoast--;
            }
        }

        currentTrackOffset += bearingOffset;

        double servoTicksFromNeutral = (angleDelta+currentTrackOffset) * (2.0 * servoTicksPer180);

        servoAngle = neutralPosition + servoTicksFromNeutral;

        servoAngle = Range.clip(servoAngle, turretMin, turretMax);

        robot.setTurretPos(servoAngle);
    }
}