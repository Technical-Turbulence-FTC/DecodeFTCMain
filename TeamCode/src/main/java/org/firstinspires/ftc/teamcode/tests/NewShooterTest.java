package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ServoPositions;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilsv2.Flywheel;
import org.firstinspires.ftc.teamcode.utilsv2.Robot;
import org.firstinspires.ftc.teamcode.utilsv2.Shooter;
import org.firstinspires.ftc.teamcode.utilsv2.Turret;
import org.firstinspires.ftc.teamcode.utilsv2.VelocityCommander;

@Config
@TeleOp
public class NewShooterTest extends LinearOpMode {

    Robot robot;
    Flywheel flywheel;
    Turret turret;
    Shooter shooter;
    MultipleTelemetry TELE;
    Follower follower;
    VelocityCommander commander;


    public static boolean intake = true;
    public static boolean shoot = false;
    public static double intakePower = 1.0;
    public static double transferShootPower = -1;
    public static double transferIntakePower = -1;
    public static double turretPos = 0.51;
    public static double hoodPos = 0.51;
    public static double flywheel_velo = 0;

    public static double shooterP = 500, shooterI = 1, shooterD = 0, shooterF = 93;

    private enum ShootState {
        IDLE,
        WAIT_GATE,
        WAIT_PUSH
    }

    private ShootState shootState = ShootState.IDLE;
    private long timestamp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.resetInstance();

        robot = Robot.getInstance(hardwareMap);

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));

        flywheel = new Flywheel(robot);
        turret = new Turret(robot);
        commander = new VelocityCommander();

        shooter = new Shooter(
                robot,
                TELE,
                follower,
                true,
                turret,
                flywheel,
                commander
        );

        shooter.setState(Shooter.ShooterState.MANUAL);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            follower.update();

            robot.setHoodPos(hoodPos);
            shooter.setTurretPosition(turretPos);
            shooter.setFlywheelVelocity(flywheel_velo);
            double voltage = robot.voltage.getVoltage();
            flywheel.setPIDF(shooterP, shooterI, shooterD, shooterF / voltage);

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

            TELE.addData("Flywheel Velocity1", (robot.shooter1.getVelocity() * 60) / 28);
            TELE.addData("Flywheel Velocity2", (robot.shooter2.getVelocity() * 60) / 28);
            TELE.addData("Flywheel Average Velocity", flywheel.getAverageVelocity());
            TELE.addData("PIDF Coefficients", Flywheel.shooterPIDF);
            TELE.addData("Power", flywheel.getShooterPower());
            TELE.addData("Distance", shooter.getDistance());
            TELE.update();

            shooter.update();
        }
    }
}