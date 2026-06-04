package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseH;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseX;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.Color;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilsv2.*;

@TeleOp
@Config
public class TeleopV4 extends LinearOpMode {
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
        Pose start = new Pose(teleStartPoseX, teleStartPoseY, Math.toRadians(teleStartPoseH));
        follower.setStartingPose(start);

        flywheel = new Flywheel(robot);
        turret = new Turret(robot);

        parkTilter = new ParkTilter(robot);

        shooter = new Shooter(robot, TELE, follower, Color.redAlliance, turret, flywheel, commander);
        shooter.setState(Shooter.ShooterState.TRACK_GOAL);
        spindexerTransferIntake = new SpindexerTransferIntake(robot, TELE, commander);
        spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Drivetrain
            drivetrain.drive(
                    -gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
            );

            follower.update();

            shooter.update();
            spindexerTransferIntake.update();

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
            if (gamepad1.rightBumperWasPressed()
                    && state == SpindexerTransferIntake.RapidMode.HOLD_BALLS) {

                spindexerTransferIntake.setRapidMode(
                        SpindexerTransferIntake.RapidMode.INTAKE
                );
            }

            if (gamepad1.dpad_down){
                parkTilter.park();
            } else if (gamepad1.dpad_up) {
                parkTilter.unpark();
            }

            TELE.addData("Distance From Goal", commander.getDistance());
            TELE.addData("Hood Position", commander.getHoodPredicted());
            TELE.addData("Transfer Power", robot.transfer.getPower());
            TELE.addData("Velocity RPM", commander.getPredictedRPM());

            TELE.update();
        }

    }
}
