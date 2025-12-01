package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.*;
import static org.firstinspires.ftc.teamcode.utils.PositionalServoProgrammer.*;

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

    double velo = 0.0;
    double velo1 = 0.0;

    double stamp1 = 0.0;

    double initPos1 = 0.0;

    double powPID = 0.0;

    public static boolean shoot = false;

    public static int spindexPos = 1;

    public static int intake = 1; // 1 for intake, 0 for outtake

    double stamp = 0.0;

    public static double distance = 50;
    double transferStamp = 0.0;
    int shot = 0;

    int ticker = 1;

    boolean spindexPosEqual (double spindexer) {
        return (scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) > spindexer - 0.01 &&
                scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) < spindexer + 0.01);
    }

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

        int ticker = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            ticker++;

            //Based on a distance it sets a velocity and hood
            if (AutoTrack) {
                hoodPos = hoodAnglePrediction(distance);
                vel = velPrediction(distance);
            }

            shooter.setShooterMode(flyMode);

            shooter.setManualPower(pow);

            robot.hood.setPosition(hoodPos);
            robot.turr1.setPosition(turretPos);
            robot.turr2.setPosition(1 - turretPos);

            //Automation for spindex and transfer if not intaking
            if (intake == 1) {
                robot.transfer.setPower(0);
                robot.transferServo.setPosition(transferServo_out);
                robot.intake.setPower(0.75);
                double position;
                if ((getRuntime() % 0.3) >0.15) {
                    position = spindexer_intakePos1 + 0.015;
                } else {
                    position = spindexer_intakePos1 - 0.015;
                }
                robot.spin1.setPosition(position);
                robot.spin2.setPosition(1-position);
            } else {
                robot.transfer.setPower(.75 + (powPID / 4));
                robot.intake.setPower(0);
                if (shoot) {

                    if (spindexPos == 1) {
                        robot.spin1.setPosition(spindexer_outtakeBall1);
                        robot.spin2.setPosition(1 - spindexer_outtakeBall1);
                        if (spindexPosEqual(spindexer_outtakeBall1)){
                            if (ticker == 1){
                                transferStamp = getRuntime();
                                ticker ++;
                            }
                            if (getRuntime() - transferStamp > waitTransfer) {
                                robot.transferServo.setPosition(transferServo_in);
                            } else {
                                robot.transferServo.setPosition(transferServo_out);
                            }
                        } else {
                            robot.transferServo.setPosition(transferServo_out);
                            ticker = 1;
                            transferStamp = getRuntime();
                        }

                    } else if (spindexPos == 2) {
                        robot.spin1.setPosition(spindexer_outtakeBall2);
                        robot.spin2.setPosition(1 - spindexer_outtakeBall2);
                        if (spindexPosEqual(spindexer_outtakeBall2)) {
                            if (ticker == 1){
                                transferStamp = getRuntime();
                                ticker ++;
                            }
                            if (getRuntime() - transferStamp > waitTransfer){
                                robot.transferServo.setPosition(transferServo_in);
                            } else {
                                robot.transferServo.setPosition(transferServo_out);
                            }
                        } else {
                            robot.transferServo.setPosition(transferServo_out);
                            ticker = 1;
                            transferStamp = getRuntime();
                        }

                    } else if (spindexPos == 3) {
                        robot.spin1.setPosition(spindexer_outtakeBall3);
                        robot.spin2.setPosition(1 - spindexer_outtakeBall3);
                        if (spindexPosEqual(spindexer_outtakeBall3)){
                            if (ticker == 1){
                                transferStamp = getRuntime();
                                ticker ++;
                            }
                            if (getRuntime() - transferStamp > waitTransfer){
                                robot.transferServo.setPosition(transferServo_in);
                            } else {
                                robot.transferServo.setPosition(transferServo_out);
                            }
                        } else {
                            transferStamp = getRuntime();
                            robot.transferServo.setPosition(transferServo_out);
                            ticker = 1;
                        }
                    }

                } else {
                    robot.transferServo.setPosition(transferServo_out);
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
            }

            double penguin;
            if (ticker % 8 == 0) {
                penguin = shooter.getECPRPosition();
                stamp = getRuntime();
                velo1 = -60 * ((penguin - initPos1) / (stamp - stamp1));
                initPos1 = penguin;
                stamp1 = stamp;
            }

            velo = velo1;

            double feed = vel / maxVel;        // Example: vel=2500 → feed=0.5

            if (vel > 500) {
                feed = Math.log((668.39 / (vel + 591.96)) - 0.116) / -4.18;
            }

            // --- PROPORTIONAL CORRECTION ---
            double error = vel - velo1;
            double correction = kP * error;

            // limit how fast power changes (prevents oscillation)
            correction = Math.max(-maxStep, Math.min(maxStep, correction));

            // --- FINAL MOTOR POWER ---
            powPID = feed + correction;

            // clamp to allowed range
            powPID = Math.max(0, Math.min(1, powPID));

            if (vel - velo > 1000) {
                powPID = 1;
            } else if (velo - vel > 1000) {
                powPID = 0;
            }

            robot.shooter1.setPower(powPID);
            robot.shooter2.setPower(powPID);

            shooter.update();

            TELE.addData("Revolutions", shooter.getECPRPosition());
            TELE.addData("hoodPos", shooter.gethoodPosition());
            TELE.addData("turretPos", shooter.getTurretPosition());
            TELE.addData("Power Fly 1", robot.shooter1.getPower());
            TELE.addData("Power Fly 2", robot.shooter2.getPower());
            TELE.addData("powPID", shooter.getpowPID());
            TELE.addData("Velocity", velo);
            TELE.update();

        }

    }

    public double hoodAnglePrediction(double distance) {
        double L = 0.298317;
        double A = 1.02124;
        double k = 0.0157892;
        double n = 3.39375;

        double dist = Math.sqrt(distance * distance + 24 * 24);

        return L + A * Math.exp(-Math.pow(k * dist, n));
    }

    public static double velPrediction(double distance) {

        double x = Math.sqrt(distance * distance + 24 * 24);

        double A = -211149.992;
        double B = -1.19943;
        double C = 3720.15909;

        return A * Math.pow(x, B) + C;
    }

}
