package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class Turret {

    public static double turrDefault = 0.4;
    public static double turretRange = 0.6;
    public static double turrMin = 0.2;
    public static double turrMax = 0.8;

    public static double turretTolerance = 0.02;

    public static double cameraBearingEqual = 1.5;
    public static double errorLearningRate = 0.02; // must be low
    public static double maxOffsetDeg = 30.0;

    private final Robot robot;
    private final MultipleTelemetry TELE;
    private final AprilTagWebcam webcam;

    private int obeliskID = 0;
    private double offsetDeg = 0.0;

    public Turret(Robot rob, MultipleTelemetry tele, AprilTagWebcam cam) {
        this.robot = rob;
        this.TELE = tele;
        this.webcam = cam;
    }

    public double getTurretPos() {
        return robot.turr1Pos.getVoltage() / 3.3;
    }

    public void manualSetTurret(double pos) {
        pos = Range.clip(pos, turrMin, turrMax);
        robot.turr1.setPosition(pos);
        robot.turr2.setPosition(1.0 - pos);
    }

    public boolean turretAt(double pos) {
        return Math.abs(pos - getTurretPos()) < turretTolerance;
    }

    public double getBearingDeg() {
        AprilTagDetection tag =
                redAlliance ? webcam.getTagById(24) : webcam.getTagById(20);
        return (tag != null) ? tag.ftcPose.bearing : Double.NaN;
    }

    public int detectObelisk() {
        if (webcam.getTagById(21) != null) obeliskID = 21;
        else if (webcam.getTagById(22) != null) obeliskID = 22;
        else if (webcam.getTagById(23) != null) obeliskID = 23;
        return obeliskID;
    }

    public int getObeliskID() {
        return obeliskID;
    }

    public void trackGoal(Pose2d deltaPos) {

        double turretAngleDeg = Math.toDegrees(
                Math.atan2(deltaPos.position.y, deltaPos.position.x)
        );

        double bearingDeg = getBearingDeg();

        if (!Double.isNaN(bearingDeg) &&
                Math.abs(bearingDeg) < cameraBearingEqual) {

            offsetDeg -= bearingDeg * errorLearningRate;
            offsetDeg = Range.clip(offsetDeg, -maxOffsetDeg, maxOffsetDeg);
        }

        turretAngleDeg += offsetDeg;

        double turretPos =
                turrDefault + (turretAngleDeg / 180.0) * turretRange;

        turretPos = Range.clip(turretPos, turrMin, turrMax);

        robot.turr1.setPosition(turretPos);
        robot.turr2.setPosition(1.0 - turretPos);

        TELE.addData("Turret Angle (deg)", turretAngleDeg);
        TELE.addData("Offset (deg)", offsetDeg);
        TELE.addData("Tag Bearing (deg)", bearingDeg);
        TELE.addData("Turret Servo", turretPos);
    }
}
