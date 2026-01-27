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
    public static double turrMin = 0.15;
    public static double turrMax = 0.85;
    public static double proportionalGain = 0.75;

    public static double visionCorrectionGain = 0.08;  // Single tunable gain
    public static double maxOffsetChangePerCycle = 5.0; // Degrees per cycle
    public static double cameraBearingEqual = 0.5;      // Deadband

    // TODO: tune these values for limelight

    public static double clampTolerance = 0.03;
    Robot robot;
    MultipleTelemetry TELE;
    Limelight3A webcam;

    double tx = 0.0;
    double ty = 0.0;
    double limelightPosX = 0.0;
    double limelightPosY = 0.0;
    private boolean lockOffset = false;
    private int obeliskID = 0;

    private double offset = 0.0;

    private double permanentOffset = 0.0;

    public Turret(Robot rob, MultipleTelemetry tele, Limelight3A cam) {
        this.TELE = tele;
        this.robot = rob;
        this.webcam = cam;
        webcam.start();
        if (redAlliance) {
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
        return turrPosScalar * (robot.intake.getCurrentPosition()) + turrDefault;

    }

    public void manualSetTurret(double pos) {
        robot.turr1.setPosition(pos);
        robot.turr2.setPosition(1 - pos);
    }

    public boolean turretEqual(double pos) {
        return Math.abs(pos - this.getTurrPos()) < turretTolerance;
    }

    private void limelightRead() { // only for tracking purposes, not general reads
        if (redAlliance) {
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
                if (botpose != null) {
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

    public double getTy() {
        limelightRead();
        return ty;
    }

    public double getLimelightX() {
        limelightRead();
        return limelightPosX;
    }

    public double getLimelightY() {
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


        /* ---------------- LIMELIGHT VISION CORRECTION ---------------- */
        double tagBearingDeg = getBearing();  // + = target is to the left
        boolean hasValidTarget = (tagBearingDeg != 1000.0);

        turretAngleDeg += permanentOffset;

        // Active correction if we see the target
        if (hasValidTarget && !lockOffset) {
            double bearingError = Math.abs(tagBearingDeg);

            if (bearingError > cameraBearingEqual) {
                // Apply sqrt scaling to reduce aggressive corrections at large errors
                double filteredBearing = Math.signum(tagBearingDeg) * Math.sqrt(Math.abs(tagBearingDeg));

                // Calculate correction
                double offsetChange = visionCorrectionGain * filteredBearing;

                // Limit rate of change to prevent jumps
                offsetChange = Math.max(-maxOffsetChangePerCycle,
                        Math.min(maxOffsetChangePerCycle, offsetChange));

                // Accumulate the correction
                offset += offsetChange;

                TELE.addData("Bearing Error", tagBearingDeg);
                TELE.addData("Offset Change", offsetChange);
                TELE.addData("Total Offset", offset);
            } else {
                // When centered, lock in the learned offset
                permanentOffset = offset;
                offset = 0.0;
            }
        }

// Apply accumulated offset
        turretAngleDeg += offset;


        /* ---------------- ANGLE → SERVO POSITION ---------------- */

        double targetTurretPos = turrDefault + (turretAngleDeg * (turret180Range * 2.0) / 360);

        // Clamp to physical servo limits
        targetTurretPos = Math.max(turrMin, Math.min(targetTurretPos, turrMax));

        // Interpolate towards target position
        double currentPos = getTurrPos();
        double turretPos = currentPos + (proportionalGain*(targetTurretPos - currentPos)); // added

        if (targetTurretPos == turrMin) {
            turretPos = turrMin;
        } else if (targetTurretPos == turrMax) {
            turretPos = turrMax;
        }

        // Set servo positions
        robot.turr1.setPosition(turretPos);
        robot.turr2.setPosition(1.0 - turretPos);


        /* ---------------- TELEMETRY ---------------- */

        TELE.addData("Turret Angle (deg)", "%.2f", turretAngleDeg);
        TELE.addData("Target Pos", "%.3f", targetTurretPos);
        TELE.addData("Current Pos", "%.3f", currentPos);
        TELE.addData("Commanded Pos", "%.3f", turretPos);
        TELE.addData("Bearing Error", hasValidTarget ? String.format("%.2f", tagBearingDeg) : "NO TARGET");
        TELE.addData("Learned Offset", "%.2f", offset);
    }

}
