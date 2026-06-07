package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utilsv2.Turret.limelightUsed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.Color;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.TeleopV4;
import org.firstinspires.ftc.teamcode.utilsv2.*;

@TeleOp
@Config
public class NewShooterTest extends LinearOpMode {
    Robot robot;
    Drivetrain drivetrain;
    Shooter shooter;
    MultipleTelemetry TELE;
    Follower follower;
    SpindexerTransferIntake spindexerTransferIntake;
    Turret turret;
    Flywheel flywheel;
    VelocityCommander commander;
    ParkTilter parkTilter;

    public static int flywheelVelo = 0;
    public static double hoodPos = 0.5;
    public static double transferPower = -0.8;
//    public static double turretPos = 0.51;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.resetInstance();

        robot = Robot.getInstance(hardwareMap);

        TELE = new MultipleTelemetry(
                FtcDashboard.getInstance().getTelemetry(), telemetry
        );

        commander = new VelocityCommander();
        drivetrain = new Drivetrain(robot, TELE);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        sleep(500);
        follower.setPose(new Pose(0,0,0));

        flywheel = new Flywheel(robot);
        turret = new Turret(robot);

        parkTilter = new ParkTilter(robot);
        parkTilter.unpark();

        shooter = new Shooter(robot, TELE, follower, Color.redAlliance, turret, flywheel, commander);
        shooter.setState(Shooter.ShooterState.MANUAL_FLYWHEEL_TRACK_TURR);
        shooter.setRedAlliance(Color.redAlliance);
        spindexerTransferIntake = new SpindexerTransferIntake(robot, TELE, commander);
        spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);

        turret.switchPipeline(Turret.PipelineMode.TRACKING);
        robot.limelight.start();

        limelightUsed = true;

        TELE.addLine("Initialization Complete");
        TELE.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Drivetrain
            drivetrain.drive(
                    -gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
            );

            if (gamepad1.crossWasPressed()){
                if (Color.redAlliance){
                    TeleopV4.relocalizePose = new Pose(56, 11, 0);
                } else {
                    TeleopV4.relocalizePose = new Pose(-56, 11, 180);
                }
                follower.setPose(TeleopV4.relocalizePose);
                gamepad1.rumble(100);
            }

            follower.update();

            if (gamepad1.dpadLeftWasPressed()){
                shooter.setState(Shooter.ShooterState.MANUAL_FLYWHEEL_TRACK_TURR);
            }

            if (gamepad1.dpadRightWasPressed()){
                shooter.setState(Shooter.ShooterState.TRACK_GOAL);
            }

            if (shooter.getState() == Shooter.ShooterState.MANUAL_FLYWHEEL_TRACK_TURR || shooter.getState() == Shooter.ShooterState.MANUAL){
                shooter.setFlywheelVelocity(flywheelVelo);
                robot.setHoodPos(hoodPos);
                robot.setTransferPower(transferPower);
            }


//            shooter.setTurretPosition(turretPos);
            shooter.update(robot.voltage.getVoltage());
            spindexerTransferIntake.update();

            if (gamepad2.leftBumperWasPressed()){
                limelightUsed = false;
            } else if (gamepad2.rightBumperWasPressed()){
                limelightUsed = true;
            }

            SpindexerTransferIntake.RapidMode state = spindexerTransferIntake.getRapidState();

            if (gamepad1.leftBumperWasPressed() &&
                    (state == SpindexerTransferIntake.RapidMode.INTAKE ||
                            state == SpindexerTransferIntake.RapidMode.TRANSFER_OFF ||
                            state == SpindexerTransferIntake.RapidMode.BEFORE_PULSE_OUT ||
                            state == SpindexerTransferIntake.RapidMode.PULSE_OUT ||
                            state == SpindexerTransferIntake.RapidMode.PULSE_IN ||
                            state == SpindexerTransferIntake.RapidMode.HOLD_BALLS)) {

                spindexerTransferIntake.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
            }

            if (gamepad1.right_trigger > 0.5 &&
                    (state == SpindexerTransferIntake.RapidMode.INTAKE ||
                            state == SpindexerTransferIntake.RapidMode.TRANSFER_OFF)) {

                spindexerTransferIntake.setRapidMode(
                        SpindexerTransferIntake.RapidMode.HOLD_BALLS
                );
            }
            if (gamepad1.right_bumper && state != SpindexerTransferIntake.RapidMode.OPEN_GATE && state != SpindexerTransferIntake.RapidMode.SHOOT) {
                robot.setIntakePower(1);
            }

            if (gamepad1.dpad_down){
                parkTilter.park();
            } else if (gamepad1.dpad_up) {
                parkTilter.unpark();
            }

            TELE.addData("Distance From Goal", commander.getDistance());
            TELE.addData("Hood Position", commander.getHoodPredicted());
            TELE.addData("Transfer Power", commander.getTransferPow());
            TELE.addData("Theoretical Velocity RPM", commander.getPredictedRPM());
            TELE.addData("Actual Velocity RPM", flywheel.getAverageVelocity());
            TELE.addData("TX:", turret.getTX());

            TELE.update();
        }

    }
}
