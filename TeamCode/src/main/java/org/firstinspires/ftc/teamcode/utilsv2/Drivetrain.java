package org.firstinspires.ftc.teamcode.utilsv2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class Drivetrain {

    Robot robot;

    MultipleTelemetry telemetry;

    private static final double DEADZONE = 0.15;
    private static final double AXIS_SNAP_THRESHOLD = 0.12;

    private static final double STRAFE_MULTIPLIER = 1.2;

    public static double FORWARD_ROTATION_CORRECTION = 0;
    public static double STRAFE_ROTATION_CORRECTION = -0;

    private boolean tele = false;

    public Drivetrain(Robot rob, MultipleTelemetry TELE) {

        this.robot = rob;

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.telemetry = TELE;
    }

    public void setTelemetry(boolean input) {
        tele = input;
    }

    public void drive(double y, double x, double rx) {

        boolean snappedForward = false;
        boolean snappedStrafe = false;

        if (Math.abs(y) < DEADZONE) y = 0;
        if (Math.abs(x) < DEADZONE) x = 0;
        if (Math.abs(rx) < DEADZONE) rx = 0;

        if (Math.abs(x) < AXIS_SNAP_THRESHOLD) {
            x = 0;
            snappedForward = true;
        }

        if (Math.abs(y) < AXIS_SNAP_THRESHOLD) {
            y = 0;
            snappedStrafe = true;
        }

        x *= STRAFE_MULTIPLIER;

        double correctionRX = 0;

        if (rx == 0) {
            correctionRX += (y * FORWARD_ROTATION_CORRECTION);
            correctionRX += (x * STRAFE_ROTATION_CORRECTION);

            rx += correctionRX;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.setFrontLeftPower(frontLeftPower);
        robot.setBackLeftPower(backLeftPower);
        robot.setFrontRightPower(frontRightPower);
        robot.setBackRightPower(backRightPower);

        if (tele) {

            telemetry.addData("Forward Snap", snappedForward);
            telemetry.addData("Strafe Snap", snappedStrafe);

            telemetry.addData("Correction RX", correctionRX);

            telemetry.addData("FL", frontLeftPower);
            telemetry.addData("BL", backLeftPower);
            telemetry.addData("FR", frontRightPower);
            telemetry.addData("BR", backRightPower);

        }
    }
}