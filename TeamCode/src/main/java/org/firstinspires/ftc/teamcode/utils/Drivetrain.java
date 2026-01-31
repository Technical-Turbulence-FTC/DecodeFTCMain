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

    public void drive(double y, double x, double rx, double brake){

        if (!autoDrive) {

            x = x* 1.1; // Counteract imperfect strafing

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
