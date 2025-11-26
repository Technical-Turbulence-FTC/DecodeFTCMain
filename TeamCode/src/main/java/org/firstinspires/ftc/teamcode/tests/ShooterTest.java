package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
@Config
public class ShooterTest extends LinearOpMode {

    Robot robot;

    public static double pow = 0.0;
    public static double vel = 0.0;
    public static double mcpr = 28.0; // CPR of the motor
    public static double ecpr = 1024.0; // CPR of the encoder
    public static double hoodPos = 0.5;
    public static double posPower = 0.0;

    public static double turretPos = 0.9;

    public static double p = 0.0003, i = 0, d = 0, f = 0;

    public static String flyMode = "MANUAL";

    public static String turrMode = "MANUAL";

    double initPos = 0.0;

    double velo1 = 0.0;

    double stamp1 = 0.0;

    double initPos1 = 0.0;

    double powPID = 0.0;

    public static double maxVel = 4500;

    public static int tolerance = 300;

    MultipleTelemetry TELE;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Shooter shooter = new Shooter(robot, TELE);

        shooter.setTelemetryOn(true);

        shooter.setTurretMode(turrMode);

        shooter.setShooterMode(flyMode);

        shooter.setControllerCoefficients(p, i, d, f);

        initPos = shooter.getECPRPosition();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            shooter.setControllerCoefficients(p, i, d, f);

            shooter.setShooterMode(flyMode);

            shooter.setManualPower(pow);

            if (hoodPos != 0.501) {
                robot.hood.setPosition(hoodPos);
            }

            if (turretPos != 0.501) {
                robot.turr1.setPosition(turretPos);
                robot.turr2.setPosition(turretPos);
            }
            velo1 = -60 * ((shooter.getECPRPosition() - initPos1) / (getRuntime() - stamp1));
            stamp1 = getRuntime();
            initPos1 = shooter.getECPRPosition();
            if (Math.abs(vel - velo1) > 3 * tolerance) {
                powPID = vel / maxVel;
            } else if (vel - tolerance > velo1) {
                powPID = powPID + 0.001;
            } else if (vel + tolerance < velo1) {
                powPID = powPID - 0.001;
            }
            shooter.setVelocity(powPID);

            shooter.update();

            TELE.addData("ECPR Revolutions", shooter.getECPRPosition());
            TELE.addData("MCPR Revolutions", shooter.getMCPRPosition());
            TELE.addData("hoodPos", shooter.gethoodPosition());
            TELE.addData("turretPos", shooter.getTurretPosition());
            TELE.addData("Power Fly 1", robot.shooter1.getPower());
            TELE.addData("Power Fly 2", robot.shooter2.getPower());
            TELE.addData("powPID", shooter.getpowPID());
            TELE.addData("Velocity", velo1);
            TELE.update();

        }

    }
}
