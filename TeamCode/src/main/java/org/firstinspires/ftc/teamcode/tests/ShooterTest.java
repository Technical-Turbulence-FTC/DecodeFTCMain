package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    public static int mode = 0;
    public static double parameter = 0.0;
    // --- CONSTANTS YOU TUNE ---
    public static double hoodPos = 0.501;
    double initPos = 0.0;
    double stamp = 0.0;
    double stamp1 = 0.0;
    double ticker = 0.0;
    double currentPos = 0.0;
    double velo = 0.0;
    double velo1 = 0.0;
    double velo2 = 0.0;
    double velo3 = 0.0;
    double velo4 = 0.0;
    double velo5 = 0.0;
    double initPosq = 0.0;
    double stampq = 0.0;
    double stamp1q = 0.0;
    double tickerq = 0.0;
    double currentPosq = 0.0;
    double veloq = 0.0;
    double velo1q = 0.0;
    double velo2q = 0.0;
    double velo3q = 0.0;
    double velo4q = 0.0;
    double velo5q = 0.0;
    Robot robot;
    Flywheel flywheel;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        DcMotorEx leftShooter = robot.shooter1;
        DcMotorEx rightShooter = robot.shooter2;
        flywheel = new Flywheel();

        MultipleTelemetry TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (mode == 0) {
                rightShooter.setPower(parameter);
                leftShooter.setPower(parameter);
                leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (ticker % 2 == 0) {
                    velo5 = velo4;
                    velo4 = velo3;
                    velo3 = velo2;
                    velo2 = velo1;

                    currentPos = (double) leftShooter.getCurrentPosition() / 28;
                    stamp = getRuntime();
                    velo1 = 60 * ((currentPos - initPos) / (stamp - stamp1));
                    initPos = currentPos;
                    stamp1 = stamp;

                    velo = (velo1 + velo2 + velo3 + velo4 + velo5) / 5;
                }
                if (tickerq % 2 == 0) {
                    velo5q = velo4q;
                    velo4q = velo3q;
                    velo3q = velo2q;
                    velo2q = velo1q;

                    currentPosq = (double) rightShooter.getCurrentPosition() / 28;
                    stampq = getRuntime();
                    velo1q = 60 * ((currentPosq - initPosq) / (stampq - stamp1q));
                    initPosq = currentPosq;
                    stamp1q = stampq;

                    veloq = (velo1q + velo2q + velo3q + velo4q + velo5q) / 5;
                }
            } else if (mode == 1) {
                leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double powPID = flywheel.manageFlywheel((int) parameter, robot.shooter1.getCurrentPosition());
                rightShooter.setPower(powPID);
                leftShooter.setPower(powPID);
                TELE.addData("PIDPower", powPID);
            }

            if (hoodPos != 0.501) {
                robot.hood.setPosition(hoodPos);
            }

            TELE.addData("Velocity1", velo);
            TELE.addData("Velocity2", veloq);
            TELE.addData("Encoder Velocity", flywheel.getVelo());
            TELE.addData("Power", robot.shooter1.getPower());
            TELE.addData("Steady?", flywheel.getSteady());
            TELE.addData("Position1", robot.shooter1.getCurrentPosition()/28);
            TELE.addData("Position2", robot.shooter2.getCurrentPosition()/28);

            TELE.update();

        }

    }
}