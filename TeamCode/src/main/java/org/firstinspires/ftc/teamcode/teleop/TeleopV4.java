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

    @Override
    public void runOpMode() throws InterruptedException {

        robot = Robot.getInstance(hardwareMap);

        TELE = new MultipleTelemetry(
                FtcDashboard.getInstance().getTelemetry(), telemetry
        );

        drivetrain = new Drivetrain(robot, TELE);
        follower = Constants.createFollower(hardwareMap);
        Pose start = new Pose(teleStartPoseX, teleStartPoseY, Math.toRadians(teleStartPoseH));
        follower.setStartingPose(start);

        shooter = new Shooter(robot, TELE, follower, Color.redAlliance);
        shooter.setState(Shooter.ShooterState.TRACK_GOAL);
        spindexerTransferIntake = new SpindexerTransferIntake(robot, TELE);
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

            shooter.update();
            spindexerTransferIntake.update();

            SpindexerTransferIntake.RapidMode state = spindexerTransferIntake.getRapidState();

            if (gamepad1.xWasPressed() &&
                    (state == SpindexerTransferIntake.RapidMode.INTAKE ||
                            state == SpindexerTransferIntake.RapidMode.TRANSFER_OFF ||
                            state == SpindexerTransferIntake.RapidMode.BEFORE_PULSE_OUT ||
                            state == SpindexerTransferIntake.RapidMode.PULSE_OUT ||
                            state == SpindexerTransferIntake.RapidMode.PULSE_IN ||
                            state == SpindexerTransferIntake.RapidMode.HOLD_BALLS)) {

                spindexerTransferIntake.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
            }

            if (gamepad1.aWasPressed() &&
                    (state == SpindexerTransferIntake.RapidMode.INTAKE ||
                            state == SpindexerTransferIntake.RapidMode.TRANSFER_OFF)) {

                spindexerTransferIntake.setRapidMode(
                        SpindexerTransferIntake.RapidMode.HOLD_BALLS
                );
            }

            if (gamepad1.yWasPressed()
                    && state == SpindexerTransferIntake.RapidMode.HOLD_BALLS) {

                spindexerTransferIntake.setRapidMode(
                        SpindexerTransferIntake.RapidMode.INTAKE
                );
            }


            TELE.update();
        }

    }
}
