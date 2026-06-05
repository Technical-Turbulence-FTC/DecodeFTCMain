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
import org.firstinspires.ftc.teamcode.utils.MeasuringLoopTimes;
import org.firstinspires.ftc.teamcode.utilsv2.*;

import static org.firstinspires.ftc.teamcode.utilsv2.Turret.limelightUsed;

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
    MeasuringLoopTimes loopTimes;

    public static Pose relocalizePose = new Pose(128, 83, 0);

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
        parkTilter.unpark();

        loopTimes = new MeasuringLoopTimes();
        loopTimes.init();

        shooter = new Shooter(robot, TELE, follower, Color.redAlliance, turret, flywheel, commander);
        shooter.setState(Shooter.ShooterState.TRACK_GOAL);
        shooter.setRedAlliance(Color.redAlliance);
        spindexerTransferIntake = new SpindexerTransferIntake(robot, TELE, commander);
        spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);

        turret.switchPipeline(Turret.PipelineMode.TRACKING);
        robot.limelight.start();

        limelightUsed = true;

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
                follower.setPose(relocalizePose);
                gamepad1.rumble(100);
            }

            follower.update();
            Pose currentPose = follower.getPose();

            if (gamepad1.dpadLeftWasPressed()){
                shooter.setState(Shooter.ShooterState.MANUAL_FLYWHEEL_TRACK_TURR);
            }

            if (gamepad1.dpadRightWasPressed()){
                shooter.setState(Shooter.ShooterState.TRACK_GOAL);
            }

            if (shooter.getState() == Shooter.ShooterState.MANUAL_FLYWHEEL_TRACK_TURR || shooter.getState() == Shooter.ShooterState.MANUAL){
                shooter.setFlywheelVelocity(2500);
                robot.setHoodPos(0.6);
                robot.setTransferPower(-0.8);
            }

            shooter.update(robot.voltage.getVoltage());
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

            if (gamepad1.right_bumper && state != SpindexerTransferIntake.RapidMode.OPEN_GATE && state != SpindexerTransferIntake.RapidMode.SHOOT) {
                robot.setIntakePower(1);
            }

            if (gamepad2.leftBumperWasPressed()){
                limelightUsed = false;
            } else if (gamepad2.rightBumperWasPressed()){
                limelightUsed = true;
            }

            if (gamepad1.dpad_down){
                parkTilter.park();
            } else if (gamepad1.dpad_up) {
                parkTilter.unpark();
            }

            loopTimes.loop();
            TELE.addData("Loop Time Average", loopTimes.getAvgLoopTime());
            TELE.addData("Loop Time Max", loopTimes.getMaxLoopTimeOneMin());
            TELE.addData("Loop Time Min", loopTimes.getMinLoopTimeOneMin());

            TELE.addData("Distance From Goal", commander.getDistance());
            TELE.addData("Hood Position", commander.getHoodPredicted());
            TELE.addData("Transfer Power", robot.transfer.getPower());
            TELE.addData("Theoretical Velocity RPM", commander.getPredictedRPM());
            TELE.addData("Actual Velocity RPM", flywheel.getAverageVelocity());

            TELE.addData("Current Position", currentPose);

            TELE.addData("Current LL Pipeline", turret.pipeline());

            TELE.update();
        }

    }
}
