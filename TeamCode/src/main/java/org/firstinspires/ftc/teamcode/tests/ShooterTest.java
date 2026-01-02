package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.ShooterVars.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {

    public static int mode = 0;
    public static double parameter = 0.0;
    // --- CONSTANTS YOU TUNE ---
    public static double MAX_RPM = 4500;      // your measured max RPM

    //TODO: @Daniel FIX THE BELOW CONSTANTS A LITTLE IF NEEDED
    public static double transferPower = 0.0;
    public static double hoodPos = 0.501;

    public static double turretPos = 0.501;
    Robot robot;
    private double lastEncoderRevolutions = 0.0;
    private double lastTimeStamp = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        DcMotorEx leftShooter = robot.shooter1;
        DcMotorEx rightShooter = robot.shooter2;
        DcMotorEx encoder = robot.shooter1;

        MultipleTelemetry TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double kF = 1.0 / MAX_RPM;     // baseline feedforward

            double encoderRevolutions = (double) encoder.getCurrentPosition() / 2048;

            double velocity = -60 * (encoderRevolutions - lastEncoderRevolutions) / (getRuntime() - lastTimeStamp);

            TELE.addLine("Mode: 0 = Manual, 1 = Vel, 2 = Pos");
            TELE.addLine("Parameter = pow, vel, or pos");
            TELE.addData("leftShooterPower", leftShooter.getPower());
            TELE.addData("rightShooterPower", rightShooter.getPower());
            TELE.addData("shaftEncoderPos", encoderRevolutions);
            TELE.addData("shaftEncoderVel", velocity);

            double velPID;

            if (mode == 0) {
                rightShooter.setPower(parameter);
                leftShooter.setPower(parameter);
            } else if (mode == 1) {

                // --- FEEDFORWARD BASE POWER ---
                double feed = kF * parameter;        // Example: vel=2500 → feed=0.5

                // --- PROPORTIONAL CORRECTION ---
                double error = parameter - velocity;
                double correction = kP * error;

                // limit how fast power changes (prevents oscillation)
                correction = Math.max(-maxStep, Math.min(maxStep, correction));

                // --- FINAL MOTOR POWER ---
                velPID = feed + correction;

                // clamp to allowed range
                velPID = Math.max(0, Math.min(1, velPID));

                rightShooter.setPower(velPID);
                leftShooter.setPower(velPID);

            }

            lastTimeStamp = getRuntime();
            lastEncoderRevolutions = (double) encoder.getCurrentPosition() / 2048;

            if (hoodPos != 0.501) {
                robot.hood.setPosition(hoodPos);
            }

            if (turretPos!=0.501){
                robot.turr1.setPosition(turretPos);
                robot.turr2.setPosition(turretPos);
            }

            robot.transfer.setPower(transferPower);

            TELE.update();

        }

    }
}
