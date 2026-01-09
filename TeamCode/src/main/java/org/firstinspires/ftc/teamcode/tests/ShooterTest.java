package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
            } else if (mode == 1) {
                double powPID = flywheel.manageFlywheel((int) parameter, leftShooter.getCurrentPosition(), rightShooter.getCurrentPosition());
                rightShooter.setPower(powPID);
                leftShooter.setPower(powPID);
                TELE.addData("PIDPower", powPID);
            }

            if (hoodPos != 0.501) {
                robot.hood.setPosition(hoodPos);
            }

            TELE.addData("Used Velocity", flywheel.getVelo(leftShooter.getCurrentPosition(), rightShooter.getCurrentPosition()));
            TELE.addData("Velocity1", flywheel.getVelo1());
            TELE.addData("Velocity2", flywheel.getVelo2());
            TELE.addData("Power", robot.shooter1.getPower());
            TELE.addData("Steady?", flywheel.getSteady());
            TELE.addData("Position1", robot.shooter1.getCurrentPosition()/28);
            TELE.addData("Position2", robot.shooter2.getCurrentPosition()/28);

            TELE.update();

        }

    }
}