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
    public static double pos = 0.0;
    public static double posPower = 0.0;

    public static double posi = 0.5;

    public static double p = 0.0003, i = 0, d = 0, f = 0;

    public static String flyMode = "MANUAL";

    public static String turrMode = "MANUAL";

    public static int posTolerance = 40;

    public static double servoPosition = 0.501;

    public double stamp = 0.0;

    public double initPos = 0.0;

    public static double time = 0.1;

    public double velo = 0.0;

    public double velo1 = 0.0;

    public double stamp1 = 0.0;

    public double initPos1 = 0.0;

    double powPID = 0.0;

    public static int tolerance = 300;

    MultipleTelemetry TELE;

    public boolean wait(double time) {
        return (getRuntime() - stamp) > time;
    }

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

            shooter.setTurretMode(turrMode);

            shooter.setShooterMode(flyMode);

            shooter.setManualPower(pow);

            shooter.sethoodPosition(pos);

            shooter.setTurretPosition(posi);

            shooter.setTolerance(posTolerance);

            shooter.setPosPower(posPower);

            if (servoPosition != 0.501) { shooter.sethoodPosition(servoPosition); }

            velo1 = -60 * ((shooter.getECPRPosition() - initPos1) / (getRuntime() - stamp1));
            stamp1 = getRuntime();
            initPos1 = shooter.getECPRPosition();
            if (Math.abs(vel - velo1) > 1500){
                if (vel - velo1 > 0){
                    powPID = 0.75;
                } else if (vel - velo1 < 0){
                    powPID = 0.25;
                }
            } else if (vel - tolerance > velo1) {
                powPID = powPID + 0.001;
            } else if (vel + tolerance < velo1) {
                powPID = powPID - 0.001;
            }
            shooter.setVelocity(powPID);

            shooter.update();

            TELE.addData("ECPR Revolutions", shooter.getECPRPosition());
            TELE.addData("MCPR Revolutions", shooter.getMCPRPosition());
            TELE.addData("Velocity", velo);
            TELE.addData("hoodPos", shooter.gethoodPosition());
            TELE.addData("turretPos", shooter.getTurretPosition());
            TELE.addData("Power Fly 1", robot.shooter1.getPower());
            TELE.addData("Power Fly 2", robot.shooter2.getPower());
            TELE.addData("powPID", shooter.getpowPID());
            TELE.addData("Ins Velocity", velo1);
            TELE.update();

        }

    }
}
