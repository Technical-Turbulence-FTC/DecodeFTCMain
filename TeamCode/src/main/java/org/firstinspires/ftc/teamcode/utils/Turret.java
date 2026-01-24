package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
public class Turret {

    public static double turretTolerance = 0.02;
    public static double turrPosScalar = 1.009;
    public static double turret180Range = 0.4;
    public static double turrDefault = 0.4;
    public static double cameraBearingEqual = 1;
    public static double errorLearningRate = 0.15;
    public static double turrMin = 0.2;
    public static double turrMax = 0.8;
    public static double deltaAngleThreshold = 0.02;
    public static double angleMultiplier = 0.0;
    Robot robot;
    MultipleTelemetry TELE;
    Limelight3A webcam;
    private int obeliskID = 0;
    private double turrPos = 0.0;
    private double offset = 0.0;
    private double bearing = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double limelightPosX = 0.0;
    double limelightPosY = 0.0;



    public Turret(Robot rob, MultipleTelemetry tele, Limelight3A cam) {
        this.TELE = tele;
        this.robot = rob;
        this.webcam = cam;
        webcam.start();
    }

    public double getTurrPos() {
        return turrPosScalar * (robot.turr1Pos.getVoltage() / 3.3);

    }

    public void manualSetTurret(double pos){
        robot.turr1.setPosition(pos);
        robot.turr2.setPosition(1-pos);
    }

    public boolean turretEqual(double pos) {
        return Math.abs(pos - this.getTurrPos()) < turretTolerance;
    }

    private void limelightRead(){ // only for tracking purposes, not general reads
        if (redAlliance){
            webcam.pipelineSwitch(3);
        } else {
            webcam.pipelineSwitch(2);
        }

        LLResult result = webcam.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                tx = result.getTx();
                ty = result.getTy();
                // MegaTag1 code for receiving position
                Pose3D botpose = result.getBotpose();
                if (botpose != null){
                    limelightPosX = botpose.getPosition().x;
                    limelightPosY = botpose.getPosition().y;
                }

            }
        }
    }

    public double getBearing() {
        tx = 1000;
        limelightRead();
        return tx;
    }

    public double getTy(){
        limelightRead();
        return ty;
    }

    public double getLimelightX(){
        limelightRead();
        return limelightPosX;
    }

    public double getLimelightY(){
        limelightRead();
        return limelightPosY;
    }

    public int detectObelisk() {
        webcam.pipelineSwitch(1);
        LLResult result = webcam.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                obeliskID = fiducial.getFiducialId();
            }
        }
        return obeliskID;
    }

    public int getObeliskID() {
        return obeliskID;
    }

    /*
        Param @deltaPos = Pose2d when subtracting robot x, y, heading from goal x, y, heading
     */
    public void trackGoal(Pose2d deltaPos) {

        /* ---------------- FIELD → TURRET GEOMETRY ---------------- */

        // Angle from robot to goal in robot frame
        double desiredTurretAngleDeg = Math.toDegrees(
                Math.atan2(deltaPos.position.y, deltaPos.position.x)
        );

        // Robot heading (field → robot)
        double robotHeadingDeg = Math.toDegrees(deltaPos.heading.toDouble());

        // Turret angle needed relative to robot
        double turretAngleDeg = desiredTurretAngleDeg - robotHeadingDeg;

        turretAngleDeg = -turretAngleDeg;

        // Normalize to [-180, 180]
        while (turretAngleDeg > 180) turretAngleDeg -= 360;
        while (turretAngleDeg < -180) turretAngleDeg += 360;


        /* ---------------- APRILTAG CORRECTION ---------------- */
//
        double tagBearingDeg = getBearing();  // + = target is to the left

        if (tagBearingDeg != 1000.0 && Math.abs(tagBearingDeg) > cameraBearingEqual) {
            // Slowly learn turret offset (persistent calibration)
            offset -= tagBearingDeg * errorLearningRate;
        }

        turretAngleDeg += offset;

        /* ---------------- ANGLE → SERVO ---------------- */

        double turretPos = turrDefault + (turretAngleDeg * (turret180Range * 2.0) / 360);

        // Clamp to servo range
        turretPos = Math.max(turrMin, Math.min(turretPos, turrMax));

        robot.turr1.setPosition(turretPos);
        robot.turr2.setPosition(1.0 - turretPos);

        /* ---------------- TELEMETRY ---------------- */

        TELE.addData("Turret Angle", turretAngleDeg);
        TELE.addData("Bearing", tagBearingDeg);
        TELE.addData("Offset", offset);
    }

}
