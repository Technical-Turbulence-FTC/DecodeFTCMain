package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
@Config
public class ShooterTest extends LinearOpMode {

    Robot robot;

    public static double pow = 0.0;
    public static double vel = 0.0;
    public static double ecpr = 1024.0; // CPR of the encoder
    public static double hoodPos = 0.5;
    public static double turretPos = 0.9;

    public static String flyMode = "VEL";

    public static boolean AutoTrack = false;

    double initPos = 0.0;

    double velo1 = 0.0;

    double stamp1 = 0.0;

    double initPos1 = 0.0;

    double powPID = 0.0;

    public static int maxVel = 4000;

    public static boolean shoot = false;

    public static int spindexPos = 1;

    public static boolean intake = true;

    public static int tolerance = 50;

    public static double kP = 0.0005;           // small proportional gain (tune this)
    public static double maxStep = 0.06;         // prevents sudden jumps
    public static double distance = 50;

    MultipleTelemetry TELE;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Shooter shooter = new Shooter(robot, TELE);

        robot.shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setTelemetryOn(true);

        shooter.setShooterMode(flyMode);

        initPos = shooter.getECPRPosition();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (AutoTrack){
                hoodPos = hoodAnglePrediction(distance);
                vel = velPrediction(distance);
            }



                shooter.setShooterMode(flyMode);

                shooter.setManualPower(pow);

                robot.hood.setPosition(hoodPos);
                robot.turr1.setPosition(turretPos);
                robot.turr2.setPosition(1 - turretPos);
                if (intake) {
                    robot.transfer.setPower(0);
                    robot.intake.setPower(0.75);
                    robot.spin1.setPosition(spindexer_intakePos);
                    robot.spin2.setPosition(1 - spindexer_intakePos);
                } else {
                    robot.transfer.setPower(1);
                    robot.intake.setPower(0);
                    if (spindexPos == 1) {
                        robot.spin1.setPosition(spindexer_outtakeBall1);
                        robot.spin2.setPosition(1 - spindexer_outtakeBall1);
                    } else if (spindexPos == 2) {
                        robot.spin1.setPosition(spindexer_outtakeBall2);
                        robot.spin2.setPosition(1 - spindexer_outtakeBall2);
                    } else if (spindexPos == 3) {
                        robot.spin1.setPosition(spindexer_outtakeBall3);
                        robot.spin2.setPosition(1 - spindexer_outtakeBall3);
                    }
                }

                velo1 = -60 * ((shooter.getECPRPosition() - initPos1) / (getRuntime() - stamp1));
                stamp1 = getRuntime();
                initPos1 = shooter.getECPRPosition();

                double feed = vel / maxVel;        // Example: vel=2500 → feed=0.5

                // --- PROPORTIONAL CORRECTION ---
                double error = vel - velo1;
                double correction = kP * error;

                // limit how fast power changes (prevents oscillation)
                correction = Math.max(-maxStep, Math.min(maxStep, correction));

                // --- FINAL MOTOR POWER ---
                powPID = feed + correction;

                // clamp to allowed range
                powPID = Math.max(0, Math.min(1, powPID));
                shooter.setVelocity(powPID);

                if (shoot) {
                    robot.transferServo.setPosition(transferServo_in);
                } else {
                    robot.transferServo.setPosition(transferServo_out);
                }

                shooter.update();

                TELE.addData("Revolutions", shooter.getECPRPosition());
                TELE.addData("hoodPos", shooter.gethoodPosition());
                TELE.addData("turretPos", shooter.getTurretPosition());
                TELE.addData("Power Fly 1", robot.shooter1.getPower());
                TELE.addData("Power Fly 2", robot.shooter2.getPower());
                TELE.addData("powPID", shooter.getpowPID());
                TELE.addData("Velocity", velo1);
                TELE.update();


        }

    }

    public double hoodAnglePrediction(double distance) {
        double L = 0.298317;
        double A = 1.02124;
        double k = 0.0157892;
        double n = 3.39375;

        double dist = Math.sqrt(distance*distance+24*24);

        return L + A * Math.exp(-Math.pow(k * dist, n));
    }
    public static double velPrediction(double distance) {

        double x = Math.sqrt(distance*distance+24*24);



        double A = -211149.992;
        double B = -1.19943;
        double C = 3720.15909;

        return A * Math.pow(x, B) + C;
    }

}
