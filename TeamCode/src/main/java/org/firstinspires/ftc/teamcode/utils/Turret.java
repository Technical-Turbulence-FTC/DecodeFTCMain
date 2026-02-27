package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.constants.Color;

import java.util.ArrayList;
import java.util.List;

@Config

public class Turret {

    public static double turretTolerance = 0.02;
    public static double turrPosScalar = 0.00011264432;
    public static double turret180Range = 0.54;
    public static double turrDefault = 0.35;
    public static double turrMin = 0;
    public static double turrMax = 1;
    public static boolean limelightUsed = true;
    public static double limelightPosOffset = 5;
    public static double manualOffset = 0.0;

//    public static double visionCorrectionGain = 0.08;  // Single tunable gain
//    public static double maxOffsetChangePerCycle = 5.0; // Degrees per cycle
//    public static double cameraBearingEqual = 0.5;      // Deadband

//    public static double clampTolerance = 0.03;
    //public static double B_PID_P = 0.105, B_PID_I = 0.0, B_PID_D = 0.0125;
    public static double B_PID_P = 0.08, B_PID_I = 0.0, B_PID_D = 0.007;
    Robot robot;
    MultipleTelemetry TELE;
    Limelight3A webcam;
    double tx = 0.0;
    double ty = 0.0;
    double limelightPosX = 0.0;
    double limelightPosY = 0.0;
    LLResult result;
    public static double TARGET_POSITION_TOLERANCE = 0.5;
    public static double COLOR_OK_TOLERANCE = 2;
    boolean bearingAligned = false;
    private boolean lockOffset = false;
    private int obeliskID = 0;
    private double offset = 0.0;
    private double currentTrackOffset = 0.0;
    private double lightColor = Color.LightRed;
    private int currentTrackCount = 0;
    double permanentOffset = 0.0;
    private int prevPipeline = -1;
    PIDController bearingPID;

    public int llCoast = 0;
    public int LL_COAST_TICKS = 60;

    public Turret(Robot rob, MultipleTelemetry tele, Limelight3A cam) {
        this.TELE = tele;
        this.robot = rob;
        this.webcam = cam;
        bearingPID = new PIDController(B_PID_P, B_PID_I, B_PID_D);
    }

    public double getLightColor() {
        return lightColor;
    }

    public void zeroTurretEncoder() {
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getTurrPos() {
        return turrPosScalar * (robot.turr1Pos.getVoltage() / 3.3) + turrDefault;

    }
    private double prevTurrPos = 0;
    private boolean isFirstTurretPos = true;
    public void setTurret(double pos) {
        if (isFirstTurretPos || prevTurrPos != pos){
            robot.turr1.setPosition(pos);
            robot.turr2.setPosition(1-pos);
            isFirstTurretPos = false;
        }
        prevTurrPos = pos;
    }
    public void pipelineSwitch(int pipeline){
        if (prevPipeline != pipeline){
            robot.limelight.pipelineSwitch(pipeline);
        }
        prevPipeline = pipeline;
    }

    public boolean turretEqual(double pos) {
        return Math.abs(pos - this.getTurrPos()) < turretTolerance;
    }

    public static double alphaPosConstant = 0.3;
    private void limelightRead() { // only for tracking purposes, not general reads
        Double xPos = null;
        Double yPos = null;
        Double zPos = null;
        Double hPos = null;
        result = webcam.getLatestResult();
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
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    limelightTagPose = fiducial.getRobotPoseTargetSpace();
                    if (limelightTagPose != null){
                        xPos = limelightTagPose.getPosition().x;
                        yPos = limelightTagPose.getPosition().y;
                        zPos = limelightTagPose.getPosition().z;
                        hPos = limelightTagPose.getOrientation().getYaw();
                    }
                }

            }
        }
        if (xPos != null){
            if (zPos<0) {
                limelightTagX = (alphaPosConstant * xPos) + ((1 - alphaPosConstant) * limelightTagX);
                limelightTagY = (alphaPosConstant * yPos) + ((1 - alphaPosConstant) * limelightTagY);
                limelightTagZ = (alphaPosConstant * zPos) + ((1 - alphaPosConstant) * limelightTagZ);
                limelightTagH = (alphaPosConstant * hPos) + ((1 - alphaPosConstant) * limelightTagH);
            }
        }
    }

    public double getBearing() {
        tx = 1000;
        limelightRead();
        return tx;
    }

    public double getTy() {
        return ty;
    }

    Pose3D limelightTagPose;
    double limelightTagX = 0.0;
    double limelightTagY = 0.0;
    double limelightTagZ = 0.0;
    double limelightTagH = 0.0;
    public double getLimelightX() {
        return limelightTagX;
    }
    public double getLimelightY() {return limelightTagY;}
    public double getLimelightZ(){return limelightTagZ;}
    public double getLimelightH(){return limelightTagH;}

    public void relocalize(){
        setTurret(turrDefault);
        limelightRead();
    }

    public int detectObelisk() {
        webcam.pipelineSwitch(1);
        LLResult result = webcam.getLatestResult();
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
    private double targetTx = 0;
    public static double alphaTX = 0.5;
    private double bearingAlign(LLResult llResult) {
        double bearingOffset = 0.0;
        double tx = llResult.getTx();   // How far left or right the target is (degrees)
        targetTx = (tx*alphaTX)+(targetTx*(1-alphaTX));
//        final double MIN_OFFSET_POWER = 0.15;
//        // LL has 54.5 degree total Horizontal FOV; very edges are not useful.
//        final double HORIZONTAL_FOV_RANGE = 26.0;  // Total usable horizontal degrees from center +/-
//        final double DRIVE_POWER_REDUCTION = 2.0;

        if (abs(targetTx) < TARGET_POSITION_TOLERANCE) {
            bearingAligned = true;
            lightColor = Color.LightBlue;
        } else if (abs(targetTx) < COLOR_OK_TOLERANCE) {
            bearingAligned = false;
            lightColor = Color.LightPurple;
        }  else {
            bearingAligned = false;
            lightColor = Color.LightOrange;
        }

        // Only with valid data and if too far off target
        if (llResult.isValid() && !bearingAligned) {

            // Adjust Robot Speed based on how far the target is located
            // Only drive at half speed max
            // switched to PID but original formula left for reference in comments
            //drivePower = targetTx/HORIZONTAL_FOV_RANGE / DRIVE_POWER_REDUCTION;
            bearingOffset = -(bearingPID.calculate(targetTx, 0.0));

//            // Make sure we have enough power to actually drive the wheels
//            if (abs(bearingOffset) < MIN_OFFSET_POWER) {
//                if (bearingOffset > 0.0) {
//                    bearingOffset = MIN_OFFSET_POWER;
//                } else {
//                    bearingOffset = -MIN_OFFSET_POWER;
//                }
//
//            }
        }

        return bearingOffset;
    }

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
        // Update local limelight results
        //double tagBearingDeg = getBearing();  // + = target is to the left
        //boolean hasValidTarget = (tagBearingDeg != 1000.0);

        turretAngleDeg += permanentOffset;


        limelightRead();
        // Active correction if we see the target
        if (result.isValid() && !lockOffset && limelightUsed) {
            currentTrackOffset += bearingAlign(result);
            currentTrackCount++;

            TELE.addData("LL Tracking: ", llCoast);

            // Assume the last tracked value is always better than
            // any previous value, even if its not fully aligned.
            llCoast = LL_COAST_TICKS;
//            double bearingError = Math.abs(tagBearingDeg);
//
//            if (bearingError > cameraBearingEqual) {
//                // Apply sqrt scaling to reduce aggressive corrections at large errors
//                double filteredBearing = Math.signum(tagBearingDeg) * Math.sqrt(Math.abs(tagBearingDeg));
//
//                // Calculate correction
//                double offsetChange = visionCorrectionGain * filteredBearing;
//
//                // Limit rate of change to prevent jumps
//                offsetChange = Math.max(-maxOffsetChangePerCycle,
//                        Math.min(maxOffsetChangePerCycle, offsetChange));
//
//                // Accumulate the correction
//                offset += offsetChange;
//
//                TELE.addData("Bearing Error", tagBearingDeg);
//                TELE.addData("Offset Change", offsetChange);
//                TELE.addData("Total Offset", offset);
//            } else {
//                // When centered, lock in the learned offset
//                permanentOffset = offset;
//                offset = 0.0;
//            }
        } else {
            // only store perma update after 20+ successful tracks
            // this did not work good in testing; only current works best so far.
//            if (currentTrackCount > 20) {
//                offset = currentTrackOffset;
//            }
            if (llCoast <= 0) {
                TELE.addData("LL No Track: ", llCoast);
                lightColor = Color.LightRed;
                currentTrackOffset = 0.0;
                currentTrackCount = 0;
            } else {
                TELE.addData("LL Coasting: ", llCoast);
                llCoast--;
            }
        }

        // Apply accumulated offset
        turretAngleDeg += offset + currentTrackOffset;


        /* ---------------- ANGLE → SERVO POSITION ---------------- */

        double targetTurretPos = turrDefault + (turretAngleDeg * (turret180Range * 2.0) / 360);

        // Clamp to physical servo limits
        targetTurretPos = Math.max(turrMin, Math.min(targetTurretPos, turrMax));

        // Interpolate towards target position
//        double currentPos = getTurrPos();
        double turretPos = targetTurretPos;

        if (targetTurretPos == turrMin) {
            turretPos = turrMin;
        } else if (targetTurretPos == turrMax) {
            turretPos = turrMax;
        }

        // Set servo positions
        if (!Spindexer.whileShooting || abs(targetTx) > COLOR_OK_TOLERANCE){
            setTurret(turretPos + manualOffset);
        }


        /* ---------------- TELEMETRY ---------------- */

//        TELE.addData("Turret Angle (deg)", "%.2f", turretAngleDeg);
//        TELE.addData("Target Pos", "%.3f", targetTurretPos);
//        TELE.addData("Current Pos", "%.3f", currentPos);
//        TELE.addData("Commanded Pos", "%.3f", turretPos);
//        TELE.addData("LL Valid", result.isValid());
//        TELE.addData("LL getTx", result.getTx());
//        TELE.addData("LL Offset", offset);
//        TELE.addData("Bearing Error", hasValidTarget ? String.format("%.2f", tagBearingDeg) : "NO TARGET");
//        TELE.addData("Learned Offset", "%.2f", offset);
    }

}
