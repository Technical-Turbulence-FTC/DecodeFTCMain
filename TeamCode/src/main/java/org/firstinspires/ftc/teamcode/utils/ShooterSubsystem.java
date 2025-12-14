package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.constants.ShooterVars.kP;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.maxStep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
public class ShooterSubsystem {
    private Robot robot;
    private MultipleTelemetry telemetry;

    public static double manualVel = 3000;
    private double vel = 3000;
    private boolean autoVel = true;
    private double velo1 = 0.0;
    private double initPos = 0.0;
    private double stamp1 = 0.0;
    private int ticker = 0;

    public ShooterSubsystem(Robot robot, MultipleTelemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    /**
     * Calculate velocity prediction based on distance
     */
    public static double velPrediction(double distance) {
        if (distance < 30) {
            return 2750;
        } else if (distance > 100) {
            if (distance > 160) {
                return 4200;
            }
            return 3700;
        } else {
            // linear interpolation between 30->2750 and 100->3700
            double slope = (3700.0 - 2750.0) / (100.0 - 30);
            return Math.round(2750 + slope * (distance - 30));
        }
    }

    /**
     * Update shooter velocity and control
     * @param runtime Current runtime
     * @param distanceToGoal Distance to goal for auto velocity calculation
     */
    public void update(double runtime, double distanceToGoal) {
        // Update velocity reading
        if (ticker % 8 == 0) {
            double penguin = (double) robot.shooterEncoder.getCurrentPosition() / 2048;
            double stamp = runtime;
            velo1 = -60 * ((penguin - initPos) / (stamp - stamp1));
            initPos = penguin;
            stamp1 = stamp;
        }

        // Update target velocity
        if (autoVel) {
            vel = velPrediction(distanceToGoal);
        } else {
            vel = manualVel;
        }

        // Calculate feedforward
        double feed = vel / 4500;
        if (vel > 500) {
            feed = Math.log((668.39 / (vel + 591.96)) - 0.116) / -4.18;
        }

        // Proportional correction
        double error = vel - velo1;
        double correction = kP * error;
        correction = Math.max(-maxStep, Math.min(maxStep, correction));

        // Final motor power
        double powPID = feed + correction;
        powPID = Math.max(0, Math.min(1, powPID));

        // Handle large errors
        if (vel - velo1 > 1000) {
            powPID = 1;
        } else if (velo1 - vel > 1000) {
            powPID = 0;
        }

        telemetry.addData("PIDPower", powPID);
        telemetry.addData("vel", velo1);
        telemetry.addData("targetVel", vel);

        robot.shooter1.setPower(powPID);
        robot.shooter2.setPower(powPID);
        robot.transfer.setPower(1);

        ticker++;
    }

    /**
     * Set velocity mode based on gamepad input
     */
    public void handleVelocityInput(boolean autoButton, double stickY, double stickX) {
        if (autoButton) {
            autoVel = true;
        } else if (stickY < -0.5) {
            autoVel = false;
            manualVel = 4100;
        } else if (stickY > 0.5) {
            autoVel = false;
            manualVel = 2700;
        } else if (stickX > 0.5) {
            autoVel = false;
            manualVel = 3600;
        } else if (stickX < -0.5) {
            autoVel = false;
            manualVel = 3100;
        }
    }

    public double getCurrentVelocity() {
        return velo1;
    }

    public double getTargetVelocity() {
        return vel;
    }

    public boolean isAutoVel() {
        return autoVel;
    }

    public void stop() {
        robot.shooter1.setPower(0);
        robot.shooter2.setPower(0);
        robot.transfer.setPower(0);
    }
}
