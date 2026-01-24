
package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
public class Turret {
    public static double turretTolerance = 0.02;
    public static double turrPosScalar = 0.00011264432;
    public static double turret180Range = 0.4;
    public static double turrDefault = 0.4;
    // TODO: tune these values for limelight
// At the top with other static variables:
    public static double kP = 0.015;  // Proportional gain - tune this first
    public static double kI = 0.0005; // Integral gain - add slowly if needed
    public static double kD = 0.002;  // Derivative gain - helps prevent overshoot

    public static double kF = 0.002;  // Derivative gain - helps prevent overshoot

    public static double maxOffset = 10; // degrees - safety limit

    // Add these as instance variables:
    private double lastTagBearing = 0.0;
    private double offsetIntegral = 0.0;

    public static double cameraBearingEqual = 1;


    public static double turrMin = 0.2;
    public static double turrMax = 0.8;
    public static double mult = 0.0;
    private boolean lockOffset = false;
    Robot robot;
    MultipleTelemetry TELE;
    Limelight3A webcam;
    private int obeliskID = 0;
    private double offset = 0.0;

    private PIDFController controller = new PIDFController(kP, kI, kD, kF);
    double tx = 0.0;
    double ty = 0.0;
    double limelightPosX = 0.0;
    double limelightPosY = 0.0;
    public static double clampTolerance = 0.03;

    public Turret(Robot rob, MultipleTelemetry tele, Limelight3A cam) {
        this.TELE = tele;
        this.robot = rob;
        this.webcam = cam;
        webcam.start();
        if (redAlliance){
            webcam.pipelineSwitch(3);
        } else {
            webcam.pipelineSwitch(2);
        }
    }

    public void zeroTurretEncoder() {
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getTurrPos() {
        return turrPosScalar * (robot.turr1Pos.getVoltage() / 3.3) + turrDefault;

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

    public void zeroOffset() {
        offset = 0.0;
    }

    public void lockOffset(boolean lock) {
        lockOffset = lock;
    }

    /*
        Param @deltaPos = Pose2d when subtracting robot x, y, heading from goal x, y, heading
     */
    public void trackGoal(Pose2d deltaPos) {

        controller.setPIDF(kP, kI, kD, kF);
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

        turretAngleDeg += offset;

        /* ---------------- ANGLE → SERVO ---------------- */

        double turretPos = turrDefault + (turretAngleDeg * (turret180Range * 2.0) / 360);

        // Clamp to servo range
        double currentEncoderPos = this.getTurrPos();

        if (!turretEqual(turretPos)) {
            double diff = turretPos - currentEncoderPos;
            turretPos = turretPos + diff * mult;
        }

        if (currentEncoderPos < (turrMin + clampTolerance) || currentEncoderPos > (turrMax - clampTolerance)) {
            // Clamp to servo range
            turretPos = Math.max(turrMin, Math.min(turretPos, turrMax));
        } else { // TODO: add so it only adds error when standstill
            if (tagBearingDeg != 1000.0 && Math.abs(tagBearingDeg) > cameraBearingEqual && !lockOffset) {
                // PID-based offset correction for faster, smoother tracking

                // Proportional: respond to current error

                 offset = -controller.calculate(tagBearingDeg);



            }
        }

        robot.turr1.setPosition(turretPos);
        robot.turr2.setPosition(1.0 - turretPos);

        /* ---------------- TELEMETRY ---------------- */

        TELE.addData("Turret Angle", turretAngleDeg);
        TELE.addData("Bearing", tagBearingDeg);
        TELE.addData("Offset", offset);
    }

}
