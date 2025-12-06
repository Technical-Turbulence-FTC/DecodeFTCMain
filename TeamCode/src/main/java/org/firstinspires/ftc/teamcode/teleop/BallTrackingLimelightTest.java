package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name="Ball Tracking Limelight Test")
public class BallTrackingLimelightTest extends LinearOpMode {

    private DcMotor fl, fr, bl, br;
    private Limelight3A limelight;

    private boolean trackingEnabled = false;
    private boolean prevTriangle = false;

    public static double steerKp = 0.02;
    public static double driveKp = 0.02;
    public static double targetAreaThreshold = 5.0;
    public static double minDrivePower = 0.1;
    public static double tooCloseThresh = 9.0;
    public static double backupPower = -0.15;

    @Override
    public void runOpMode() {

        // init
        fl = hardwareMap.dcMotor.get("frontLeft");
        bl = hardwareMap.dcMotor.get("backLeft");
        fr = hardwareMap.dcMotor.get("frontRight");
        br = hardwareMap.dcMotor.get("backRight");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // wait for tri
            if (gamepad1.triangle && !prevTriangle) {
                trackingEnabled = !trackingEnabled;
            }
            prevTriangle = gamepad1.triangle;

            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x;
            double driveRot = gamepad1.right_stick_x;

            if (!trackingEnabled) {
                mecanumDrive(driveY, driveX, driveRot);
                telemetry.addData("tracking", "false");
                telemetry.update();
                continue;
            }

            LLResult result = limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                mecanumDrive(0, 0, 0);
                telemetry.addData("tracking", "false");
                telemetry.addData("ball reached", false);
                telemetry.update();
                continue;
            }

            // only tx and ta needed for straight mount
            double targetX = result.getTx();
            double targetArea = result.getTa();

            boolean ballReached = targetArea >= targetAreaThreshold;
            boolean tooClose = targetArea >= tooCloseThresh;

            // control
            if (tooClose) {
                mecanumDrive(backupPower, 0, 0);
            } else if (ballReached) {
                mecanumDrive(0, 0, 0);
            } else {
                double steer = targetX * steerKp;
                double drive = (targetAreaThreshold - targetArea) * driveKp;
                if (drive < minDrivePower) drive = minDrivePower;
                mecanumDrive(drive, 0, steer);
            }

            // tele
            telemetry.addData("track", trackingEnabled);
            telemetry.addData("reached", ballReached);
            telemetry.addData("too close", tooClose);
            telemetry.addData("tx", targetX);
            telemetry.addData("ta", targetArea);
            telemetry.update();
        }
    }

    // base 
    private void mecanumDrive(double y, double x, double rx) {
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        fl.setPower((y + x + rx) / denom);
        bl.setPower((y - x + rx) / denom);
        fr.setPower((y - x - rx) / denom);
        br.setPower((y + x - rx) / denom);
    }
}
