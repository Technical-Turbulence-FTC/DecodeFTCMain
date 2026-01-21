package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

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

    //TODO: @Daniel FIX THE BELOW CONSTANTS A LITTLE IF NEEDED
    public static double Velocity = 0.0;
    public static double P = 40.0;
    public static double I = 0.3;
    public static double D = 7.0;
    public static double F = 10.0;
    public static double transferPower = 1.0;
    public static double hoodPos = 0.501;
    public static double turretPos = 0.501;
    public static boolean shoot = false;
    Robot robot;
    Flywheel flywheel;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        DcMotorEx leftShooter = robot.shooter1;
        DcMotorEx rightShooter = robot.shooter2;
        flywheel = new Flywheel(hardwareMap);

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
                flywheel.setPIDF(P,I,D,F);
                flywheel.manageFlywheel((int) Velocity);
            }

            if (hoodPos != 0.501) {
                robot.hood.setPosition(hoodPos);
            }


            robot.transfer.setPower(transferPower);
            if (shoot) {
                robot.transferServo.setPosition(transferServo_in);
            } else {
                robot.transferServo.setPosition(transferServo_out);
            }
            TELE.addData("Velocity", flywheel.getVelo());
            TELE.addData("Velocity 1", flywheel.getVelo1());
            TELE.addData("Velocity 2", flywheel.getVelo2());
            TELE.addData("Power", robot.shooter1.getPower());
            TELE.addData("Steady?", flywheel.getSteady());
            TELE.addData("Position", robot.shooter1.getCurrentPosition());

            TELE.update();

        }

    }
}
