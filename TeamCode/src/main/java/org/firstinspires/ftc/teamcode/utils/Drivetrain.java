package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Drivetrain {
    private Robot robot;

    public Drivetrain(Robot robot) {
        this.robot = robot;
    }

    /**
     * Sets the power for mecanum drive based on gamepad inputs
     * @param y Forward/backward movement (negative because Y stick is reversed)
     * @param x Strafe left/right (multiplied by 1.1 to counteract imperfect strafing)
     * @param rx Rotation
     */
    public void setDrivePower(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.frontLeft.setPower(frontLeftPower);
        robot.backLeft.setPower(backLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.backRight.setPower(backRightPower);
    }

    public void stop() {
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }
}
