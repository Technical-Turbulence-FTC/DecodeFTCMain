package org.firstinspires.ftc.teamcode.tests;

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
public class SortedSpindexerTest extends LinearOpMode {
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
    public static String order = "GPP";

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
        shooter.setRedAlliance(Color.redAlliance);
        spindexerTransferIntake = new SpindexerTransferIntake(robot, TELE, commander);
        spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.SORTED);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            switch(order) {
                case "PPG":
                    spindexerTransferIntake.setDesiredPattern(
                            SpindexerTransferIntake.DesiredPattern.PPG
                    );
                    break;

                case "PGP":
                    spindexerTransferIntake.setDesiredPattern(
                            SpindexerTransferIntake.DesiredPattern.PGP
                    );
                    break;

                default:
                    spindexerTransferIntake.setDesiredPattern(
                            SpindexerTransferIntake.DesiredPattern.GPP
                    );
            }


            //Drivetrain
            drivetrain.drive(
                    -gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x,
                    gamepad1.left_trigger
            );

            follower.update();


            shooter.update(robot.voltage.getVoltage());
            spindexerTransferIntake.update();

            SpindexerTransferIntake.RapidMode state = spindexerTransferIntake.getRapidState();

            if(gamepad1.leftBumperWasPressed()) {
                spindexerTransferIntake.startSortedShoot();
            }

            if (gamepad1.dpad_down){
                parkTilter.park();
            } else if (gamepad1.dpad_up) {
                parkTilter.unpark();
            }

            TELE.addData("Distance From Goal", commander.getDistance());
            TELE.addData("Hood Position", commander.getHoodPredicted());
            TELE.addData("Transfer Power", robot.transfer.getPower());
            TELE.addData("Theoretical Velocity RPM", commander.getPredictedRPM());
            TELE.addData("Actual Velocity RPM", flywheel.getAverageVelocity());

            TELE.update();
        }

    }
}
