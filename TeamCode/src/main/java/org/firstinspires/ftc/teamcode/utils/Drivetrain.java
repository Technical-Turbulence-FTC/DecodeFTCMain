package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;

public class Drivetrain {

     Robot robot;
    boolean autoDrive = false;

    Pose2d brakePos = new Pose2d(0, 0, 0);

    MecanumDrive drive;

    private final TranslationalVelConstraint VEL_CONSTRAINT = new TranslationalVelConstraint(200);
    private final ProfileAccelConstraint ACCEL_CONSTRAINT = new ProfileAccelConstraint(-Math.abs(60), 200);


    public Drivetrain (Robot rob, MecanumDrive mecanumDrive){

        this.robot = rob;
        this.drive = mecanumDrive;

    }

    private double prevY = 0;
    private double prevX = 0;
    private double prevRX = 0;
    private double prevBrake = 0;
    public void drive(double y, double x, double rx, double brake){
        int countConstant = 0;
        boolean brakeChange = false;
        if (Math.abs(prevY - y) > 0.05){
            prevY = y;
            countConstant++;
        }
        if (Math.abs(prevX - x) > 0.05){
            prevX = x;
            countConstant++;
        }
        if (Math.abs(prevRX - rx) > 0.05){
            prevRX = rx;
            countConstant++;
        }
        if (Math.abs(prevBrake - brake) > 0.05){
            prevBrake = brake;
            brakeChange = true;
        }

        if (!autoDrive && countConstant > 0) {

            x = x* 1.1; // Counteract imperfect strafing


            double denominator = Math.max(Math.abs(prevY) + Math.abs(prevX) + Math.abs(prevRX), 1);
            double frontLeftPower = (prevY + prevX + prevRX) / denominator;
            double backLeftPower = (prevY - prevX + prevRX) / denominator;
            double frontRightPower = (prevY - prevX - prevRX) / denominator;
            double backRightPower = (prevY + prevX - prevRX) / denominator;

            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backRight.setPower(backRightPower);
        }

        if (brake > 0.4 && robot.frontLeft.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE && !autoDrive) {
            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            drive.updatePoseEstimate();

            brakePos = drive.localizer.getPose();
            autoDrive = true;

        } else if (brake > 0.4) {
            drive.updatePoseEstimate();

            Pose2d currentPos = drive.localizer.getPose();

            TrajectoryActionBuilder traj2 = drive.actionBuilder(currentPos)
                    .strafeToLinearHeading(new Vector2d(brakePos.position.x, brakePos.position.y), brakePos.heading.toDouble(), VEL_CONSTRAINT, ACCEL_CONSTRAINT);

            if (Math.abs(currentPos.position.x - brakePos.position.x) > 1 || Math.abs(currentPos.position.y - brakePos.position.y) > 1) {
                Actions.runBlocking(
                        traj2.build()
                );
            }
        } else {
            autoDrive = false;
            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }

    }
}
