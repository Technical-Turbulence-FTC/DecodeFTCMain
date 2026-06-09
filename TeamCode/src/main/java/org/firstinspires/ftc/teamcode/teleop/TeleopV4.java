package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.Color;
import org.firstinspires.ftc.teamcode.constants.ServoPositions;
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

    public static Pose relocalizePose = new Pose(54.7, 11.4, 0);
    public static Pose teleStart = new Pose(0,0,0);

    private boolean firstTickFull = true;
    private boolean intakeFull = true;
    private boolean shooting = false;
    public static int flywheelOffset = 0;
    public static double hoodOffset = 0;

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
        follower.setStartingPose(teleStart);
        sleep(500);
        follower.setPose(teleStart);
        follower.update();

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

        TELE.addLine("Initialization is done");
        TELE.addData("Starting Position", follower.getPose());
        TELE.addData("TELE START", teleStart);
        TELE.update();

        while (opModeInInit()){
            if (gamepad1.triangleWasPressed()){
                VelocityCommander.lockFront = true;
                VelocityCommander.lockBack = false;
                spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);
            } else if (gamepad1.squareWasPressed()){
                VelocityCommander.lockBack = true;
                VelocityCommander.lockFront = false;
                spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.SPINDEXER_BACK);
            } else if (gamepad1.circleWasPressed()){
                VelocityCommander.lockBack = false;
                VelocityCommander.lockFront = false;
                spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);
            }
            TELE.addLine("Initialization is done");
            TELE.addData("Starting Position", follower.getPose());
            TELE.addData("TELE START", teleStart);
            TELE.addData("Front?:", VelocityCommander.lockFront);
            TELE.addData("Back?:", VelocityCommander.lockBack);
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        int emptyTicks = 0;
        while (opModeIsActive()) {
            //Drivetrain
            drivetrain.drive(
                    -gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x,
                    gamepad1.left_trigger
            );

            if (gamepad1.crossWasPressed()){
                if (Color.redAlliance){
                    relocalizePose = new Pose(54.4, 12, 0);
                } else {
                    relocalizePose = new Pose(-54.4, 12, Math.toRadians(180));
                }
                follower.setPose(relocalizePose);
                sleep(500);
                gamepad1.rumble(100);
            }

            follower.update();
            Pose currentPose = follower.getPose();
            teleStart = currentPose;

            if (gamepad1.dpadLeftWasPressed()){
                shooter.setState(Shooter.ShooterState.MANUAL_FLYWHEEL_TRACK_TURR);
            }

            if (gamepad1.dpadRightWasPressed()){
                shooter.setState(Shooter.ShooterState.TRACK_GOAL);
            }

            if (shooter.getState() == Shooter.ShooterState.MANUAL_FLYWHEEL_TRACK_TURR || shooter.getState() == Shooter.ShooterState.MANUAL){
                shooter.setFlywheelVelocity(2500 + flywheelOffset);
                robot.setHoodPos(0.6 + hoodOffset);
            }

            if (gamepad1.triangleWasPressed()){
                VelocityCommander.lockFront = true;
                VelocityCommander.lockBack = false;
                spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);
                spindexerTransferIntake.setRapidMode(SpindexerTransferIntake.RapidMode.INTAKE);
                TELE.addData("Front?:", true);
                TELE.addData("Back?:", false);
            } else if (gamepad1.squareWasPressed()){
                VelocityCommander.lockBack = true;
                VelocityCommander.lockFront = false;
                spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.SPINDEXER_BACK);
                spindexerTransferIntake.setSortedIntakeMode(SpindexerTransferIntake.SortedIntakeStates.IDLE);
                TELE.addData("Front?:", false);
                TELE.addData("Back?:", true);
            } else if (gamepad1.circleWasPressed()){
                VelocityCommander.lockBack = false;
                VelocityCommander.lockFront = false;
                spindexerTransferIntake.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);
                spindexerTransferIntake.setRapidMode(SpindexerTransferIntake.RapidMode.INTAKE);
                TELE.addData("Front?:", false);
                TELE.addData("Back?:", true);
            }

            shooter.update(robot.voltage.getVoltage());
            spindexerTransferIntake.update();

            if (VelocityCommander.lockBack){
                robot.setRapidFireBlockerPos(ServoPositions.rapidFireBlocker_Open);
                if (gamepad1.leftBumperWasPressed()){
                    spindexerTransferIntake.startBackShooting();
                    firstTickFull = true;
                }

                if (spindexerTransferIntake.getSortedIntakeStates() == SpindexerTransferIntake.SortedIntakeStates.REVERSE && firstTickFull){
                    gamepad1.rumble(100);
                    firstTickFull = false;
                }
                TELE.addData("Intake State", spindexerTransferIntake.getSortedIntakeStates());
            } else {
                robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Open);
                SpindexerTransferIntake.RapidMode state = spindexerTransferIntake.getRapidState();

                if (gamepad1.leftBumperWasPressed() &&
                        (state == SpindexerTransferIntake.RapidMode.INTAKE ||
                                state == SpindexerTransferIntake.RapidMode.TRANSFER_OFF ||
                                state == SpindexerTransferIntake.RapidMode.BEFORE_PULSE_OUT ||
                                state == SpindexerTransferIntake.RapidMode.PULSE_OUT ||
                                state == SpindexerTransferIntake.RapidMode.PULSE_IN ||
                                state == SpindexerTransferIntake.RapidMode.HOLD_BALLS)) {

                    spindexerTransferIntake.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
                    intakeFull = false;
                    firstTickFull = true;
                    shooting = true;
                }

                if (state != SpindexerTransferIntake.RapidMode.OPEN_GATE && state != SpindexerTransferIntake.RapidMode.SHOOT){
                    shooting = false;
                }

                if (robot.insideBeam.isPressed() && robot.outsideBeam.isPressed() && !shooting){
                    intakeFull = true;
                } else {
                    intakeFull = false;
                    firstTickFull = true;
                }

                if (intakeFull && firstTickFull){
                    gamepad1.rumble(100);
                    firstTickFull = false;
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
                    robot.setTransferPower(-0.7);
                }
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

            if (gamepad2.leftBumperWasPressed()){
                limelightUsed = false;
            } else if (gamepad2.rightBumperWasPressed()){
                limelightUsed = true;
            }

            if (gamepad2.dpadUpWasPressed()){
                hoodOffset-=0.02;
            } else if (gamepad2.dpadDownWasPressed()){
                hoodOffset+=0.02;
            }

            if (gamepad2.dpadRightWasPressed()){
                flywheelOffset+=50;
            } else if (gamepad2.dpadLeftWasPressed()){
                flywheelOffset-=50;
            }

            if (gamepad2.crossWasPressed()){
                flywheelOffset = 0;
                hoodOffset = 0;
            }

            loopTimes.loop();
//            TELE.addData("Loop Time Average", loopTimes.getAvgLoopTime());
//            TELE.addData("Loop Time Max", loopTimes.getMaxLoopTimeOneMin());
//            TELE.addData("Loop Time Min", loopTimes.getMinLoopTimeOneMin());
//
            TELE.addData("Distance From Goal", commander.getDistance());
//            TELE.addData("Hood Position", commander.getHoodPredicted());
//            TELE.addData("Transfer Power", robot.transfer.getPower());
            TELE.addData("Theoretical Velocity RPM", commander.getPredictedRPM());
            TELE.addData("Actual Velocity RPM", flywheel.getAverageVelocity());
            TELE.addData("Velocity 1", flywheel.getVelo1());
            TELE.addData("Velocity 2", flywheel.getVelo2());
            TELE.addData("Flywheel Offset", flywheelOffset);
            TELE.addData("Hood Offset", hoodOffset);

            TELE.addData("FR Drive", robot.frontRight.getCurrent(CurrentUnit.AMPS));
            TELE.addData("FL Drive", robot.frontRight.getCurrent(CurrentUnit.AMPS));
            TELE.addData("BL Drive", robot.backLeft.getCurrent(CurrentUnit.AMPS));
            TELE.addData("BR Drive", robot.backRight.getCurrent(CurrentUnit.AMPS));

            TELE.addData("Flywheel 1", robot.shooter1.getCurrent(CurrentUnit.AMPS));
            TELE.addData("Flywheel 2", robot.shooter2.getCurrent(CurrentUnit.AMPS));
            TELE.addData("Transfer", robot.transfer.getCurrent(CurrentUnit.AMPS));
            TELE.addData("Intake", robot.intake.getCurrent(CurrentUnit.AMPS));
//
//            TELE.addData("Current Position", currentPose);
//
//            TELE.addData("Current LL Pipeline", turret.pipeline());
//
            TELE.update();
        }

    }
}
