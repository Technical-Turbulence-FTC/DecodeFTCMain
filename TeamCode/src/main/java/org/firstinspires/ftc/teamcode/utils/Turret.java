package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class Turret {

    public static double turretTolerance = 0.02;
    public static double turrPosScalar = 1;
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
    AprilTagWebcam webcam;
    private int obeliskID = 0;
    private double turrPos = 0.0;
    private double offset = 0.0;
    private double bearing = 0.0;



    public Turret(Robot rob, MultipleTelemetry tele, AprilTagWebcam cam) {
        this.TELE = tele;
        this.robot = rob;
        this.webcam = cam;
    }

    public double getTurrPos() {
        return turrPosScalar * (robot.intake.getCurrentPosition());

    }

    public void manualSetTurret(double pos){
        robot.turr1.setPosition(pos);
        robot.turr2.setPosition(1-pos);
    }

    public boolean turretEqual(double pos) {
        return Math.abs(pos - this.getTurrPos()) < turretTolerance;
    }

    public double getBearing() {
        if (redAlliance) {
            AprilTagDetection d24 = webcam.getTagById(24);
            if (d24 != null) {
                bearing = d24.ftcPose.bearing;
                return bearing;
            } else {
                return 1000.0;
            }
        } else {
            AprilTagDetection d20 = webcam.getTagById(20);
            if (d20 != null) {
                bearing = d20.ftcPose.bearing;
                return bearing;
            } else {
                return 1000.0;
            }
        }
    }

    public int detectObelisk() {
        AprilTagDetection id21 = webcam.getTagById(21);
        AprilTagDetection id22 = webcam.getTagById(22);
        AprilTagDetection id23 = webcam.getTagById(23);
        if (id21 != null) {
            obeliskID = 21;
        } else if (id22 != null) {
            obeliskID = 22;
        } else if (id23 != null) {
            obeliskID = 23;
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
