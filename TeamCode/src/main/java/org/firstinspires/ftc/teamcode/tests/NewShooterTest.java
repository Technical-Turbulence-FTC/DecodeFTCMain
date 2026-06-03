package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ServoPositions;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilsv2.Flywheel;
import org.firstinspires.ftc.teamcode.utilsv2.Robot;
import org.firstinspires.ftc.teamcode.utilsv2.Shooter;

@Config
@TeleOp
public class NewShooterTest extends LinearOpMode {

    Robot robot;

    MultipleTelemetry TELE;

    Flywheel rpmFlywheel;

    public static boolean intake = true;
    public static boolean shoot = false;
    public static double intakePower = 1.0;
    public static double transferShootPower = -1;
    public static double transferIntakePower = -1.0;
    public static double turretPos = 0.51;
    public static double hoodPos = 0.51;
    public static double flywheel = 0;

    public static double shooterP = 255, shooterI = 0, shooterD = 0, shooterF = 75;

    private enum ShootState {
        IDLE,
        WAIT_GATE,
        WAIT_PUSH
    }

    private ShootState shootState = ShootState.IDLE;
    private long timestamp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = Robot.getInstance(hardwareMap);

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Shooter shooter = new Shooter(
                robot,
                new MultipleTelemetry(
                        telemetry,
                        FtcDashboard.getInstance().getTelemetry()
                ),
                Constants.createFollower(hardwareMap),
                true
        );

        rpmFlywheel = new Flywheel(robot);

        shooter.setState(Shooter.ShooterState.MANUAL);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            robot.setHoodPos(hoodPos);
            shooter.setTurretPosition(turretPos);
            shooter.setFlywheelVelocity(flywheel);
            double voltage = robot.voltage.getVoltage();
            rpmFlywheel.setPIDF(shooterP, shooterI, shooterD, shooterF / voltage);

            robot.setSpinPos(ServoPositions.spindexer_A2);

            if (intake && !shoot) {

                shootState = ShootState.IDLE;

                robot.setRapidFireBlockerPos(
                        ServoPositions.rapidFireBlocker_Closed);

                robot.setTransferPower(transferIntakePower);
                robot.setIntakePower(intakePower);
                robot.setTransferServoPos(
                        ServoPositions.transferServo_out);
            } else if (shoot) {
                robot.setIntakePower(intakePower);


                switch (shootState) {

                    case IDLE:

                        robot.setTransferPower(transferShootPower);

                        timestamp = System.currentTimeMillis();
                        shootState = ShootState.WAIT_GATE;

                        break;

                    case WAIT_GATE:

                        if (System.currentTimeMillis() - timestamp >= 300) {

                            robot.setRapidFireBlockerPos(
                                    ServoPositions.rapidFireBlocker_Open);

                            timestamp = System.currentTimeMillis();
                            shootState = ShootState.WAIT_PUSH;
                        }

                        break;

                    case WAIT_PUSH:

                        if (System.currentTimeMillis() - timestamp >= 100) {

                            robot.setTransferServoPos(
                                    ServoPositions.transferServo_in);

                            shootState = ShootState.IDLE;
                        }

                        break;
                }
            }

            TELE.addData("Flywheel Velocity", (robot.shooter1.getVelocity() * 60) / 28);
            TELE.update();

            shooter.update();
        }
    }
}