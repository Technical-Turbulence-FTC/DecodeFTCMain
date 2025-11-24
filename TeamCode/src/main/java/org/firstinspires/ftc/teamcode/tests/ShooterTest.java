package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {

    public static int mode = 0;
    public static double parameter = 0.0;
    Robot robot;
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    private DcMotorEx encoder;
    private double encoderRevolutions = 0.0;
    private double lastEncoderRevolutions = 0.0;
    private double timeStamp = 0.0;
    private double lastTimeStamp = 0.0;


    // --- CONSTANTS YOU TUNE ---
    public static double MAX_RPM = 2500;      // your measured max RPM
    public static double kP = 0.01;           // small proportional gain (tune this)
    public static double maxStep = 0.2;         // prevents sudden jumps

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        leftShooter = robot.shooter1;
        rightShooter = robot.shooter2;
        encoder = robot.shooter1;

        MultipleTelemetry TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double kF = 1.0 / MAX_RPM;     // baseline feedforward




            encoderRevolutions = (double) encoder.getCurrentPosition() / 2048;

            double velocity = -60*(encoderRevolutions - lastEncoderRevolutions) / (getRuntime() - lastTimeStamp);

            TELE.addLine("Mode: 0 = Manual, 1 = Vel, 2 = Pos");
            TELE.addLine("Parameter = pow, vel, or pos");
            TELE.addData("leftShooterPower",  leftShooter.getPower());
            TELE.addData("rightShooterPower", rightShooter.getPower());
            TELE.addData("shaftEncoderPos", encoderRevolutions);
            TELE.addData("shaftEncoderVel", velocity);

            double velPID = 0.0;

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



            TELE.update();



        }

    }
}
