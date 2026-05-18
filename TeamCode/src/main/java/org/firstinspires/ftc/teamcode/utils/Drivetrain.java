package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {

    Robot robot;
    boolean autoDrive = false;

    Pose brakePos = new Pose(0, 0, 0);

//    MecanumDrive drive;
    Follower follower;

    private final TranslationalVelConstraint VEL_CONSTRAINT = new TranslationalVelConstraint(200);
    private final ProfileAccelConstraint ACCEL_CONSTRAINT = new ProfileAccelConstraint(-Math.abs(60), 200);


    public Drivetrain (Robot rob, Follower follower){

        this.robot = rob;
        this.follower = follower;

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
        Pose currentPos = follower.getPose();
        brakePos = currentPos;

        if (brake > 0.4 && robot.frontLeft.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE && !autoDrive) {
            robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            autoDrive = true;

        } else if (brake > 0.4) {

            PathChain traj2 = follower.pathBuilder()
                    .addPath(new BezierLine(currentPos, brakePos))
                    .setLinearHeadingInterpolation(currentPos.getHeading(), brakePos.getHeading())
                    .build();

            if (Math.abs(currentPos.getX() - brakePos.getX()) > 1 || Math.abs(currentPos.getY() - brakePos.getY()) > 1) {
                follower.followPath(traj2);
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
