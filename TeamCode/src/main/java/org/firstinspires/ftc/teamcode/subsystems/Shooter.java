package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
public class Shooter {

    // ================================================================
    // -------------------  DASHBOARD CONSTANTS  -----------------------
    // ================================================================

    public static int mode = 0;            // 0 = manual, 1 = velocity PID, 2 = autoTrack
    public static double parameter = 0.0;  // manual: power, vel: target RPM, auto: target RPM

    public static double MAX_RPM = 2500;
    public static double kP = 0.01;
    public static double maxStep = 0.2;

    public static double transferPower = 0.0;
    public static double hoodPos = 0.501;

    // ================================================================
    // -------------   AUTO TRACK TUNING CONSTANTS  -------------------
    // ================================================================

    // z offset between shooter and goal
    public static double dz = 30;   // inches

    // Quadratic fit for shooter velocity vs distance
    // v = A*d^2 + B*d + C
    public static double A = 0.0004;
    public static double B = 0.9;
    public static double C = 1200;

    // Hood angle trig model
    // hood = HOOD_A * atan(d * HOOD_B) + HOOD_C
    public static double HOOD_A = 0.42;
    public static double HOOD_B = 0.012;
    public static double HOOD_C = 0.22;

    // ================================================================
    // ----------------------  INTERNAL STATE  ------------------------
    // ================================================================

    private DcMotorEx leftShooter, rightShooter, encoder;

    private double lastRevolutions = 0.0;
    private double lastTime = 0.0;

    private MultipleTelemetry TELE;
    private Robot robot;

    // ================================================================
    // --------------------------- INIT -------------------------------
    // ================================================================

    public void init(Robot robot, MultipleTelemetry TELE) {
        this.robot = robot;
        this.TELE = TELE;

        leftShooter = robot.shooter1;
        rightShooter = robot.shooter2;
        encoder = robot.shooter1;

        lastTime = 0.0;
        lastRevolutions = 0.0;
    }

    // ================================================================
    // -------------------------- UPDATE ------------------------------
    // ================================================================

    public void update(double runtimeSec) {

        double kF = 1.0 / MAX_RPM;

        double rev = encoder.getCurrentPosition() / 2048.0;
        double velocity = -60 * (rev - lastRevolutions) / (runtimeSec - lastTime);

        TELE.addLine("Mode: 0=Manual, 1=Vel, 2=AutoTrack");
        TELE.addData("Parameter", parameter);
        TELE.addData("Velocity", velocity);

        if (mode == 0) {
            // Manual
            leftShooter.setPower(parameter);
            rightShooter.setPower(parameter);
        }
        else if (mode == 1 || mode == 2) {
            // Velocity PID (shared logic)
            double feed = kF * parameter;
            double error = parameter - velocity;
            double correction = kP * error;

            correction = Math.max(-maxStep, Math.min(maxStep, correction));

            double finalPower = Math.max(0, Math.min(1, feed + correction));

            leftShooter.setPower(finalPower);
            rightShooter.setPower(finalPower);
        }

        robot.hood.setPosition(hoodPos);
        robot.transfer.setPower(transferPower);

        lastTime = runtimeSec;
        lastRevolutions = rev;

        TELE.update();
    }

    // ================================================================
    // ------------------------ AUTO TRACK ----------------------------
    // ================================================================

    public void autoTrack(Pose2d robotPose, Pose2d goalPose) {

        mode = 2;  // Auto tracking → velocity PID

        // Compute 3D distance
        double dx = goalPose.position.x - robotPose.position.x;
        double dy = goalPose.position.y - robotPose.position.y;

        double distance = Math.sqrt(dx*dx + dy*dy + dz*dz);

        // ---- Velocity Fit ----
        double targetVelocity = A * distance * distance + B * distance + C;
        parameter = targetVelocity;

        // ---- Hood Fit ----
        hoodPos = HOOD_A * Math.atan(distance * HOOD_B) + HOOD_C;

        // Telemetry
        TELE.addLine("AUTO TRACK ACTIVE");
        TELE.addData("Distance", distance);
        TELE.addData("Target Velocity", targetVelocity);
        TELE.addData("Hood", hoodPos);
    }

    // ================================================================
    // --------------------- USER CALL METHODS ------------------------
    // ================================================================

    public void setManualPower(double p) {
        mode = 0;
        parameter = p;
    }

    public void setVelocity(double rpm) {
        mode = 1;
        parameter = rpm;
    }

    public void setHood(double pos) { hoodPos = pos; }

    public void setTransfer(double p) { transferPower = p; }
}