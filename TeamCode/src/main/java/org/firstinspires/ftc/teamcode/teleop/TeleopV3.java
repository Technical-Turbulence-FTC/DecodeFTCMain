package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Targeting.turretInterpolate;
import static org.firstinspires.ftc.teamcode.utils.Turret.limelightUsed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.util.List;

@Config
@TeleOp
public class TeleopV3 extends LinearOpMode {
    public static double manualVel = 3000;
    public static double hoodDefaultPos = 0.5;

    public static double spinPow = 0.09;
    public static double tp = 0.8, ti = 0.001, td = 0.0315, tf = 0;
    public static double spinSpeedIncrease = 0.03;
    public static int resetSpinTicks = 4;
    public static double hoodSpeedOffset = 0.01;
    public static double turretSpeedOffset = 0.01;
    public double vel = 3000;
    public boolean autoVel = true;
    public boolean targetingHood = true;
    public boolean autoHood = true;
    public double shootStamp = 0.0;
    boolean fixedTurret = false;
    Robot robot;
    MultipleTelemetry TELE;
    Servos servo;
    Flywheel flywheel;
    MecanumDrive drive;
    Spindexer spindexer;
    Targeting targeting;
    Targeting.Settings targetingSettings;
    Drivetrain drivetrain;
    double autoHoodOffset = 0.0;
    int shooterTicker = 0;
    boolean intake = false;
    boolean reject = false;
    double xOffset = 0.0;
    double yOffset = 0.0;
    double headingOffset = 0.0;
    int ticker = 0;


    boolean autoSpintake = false;
    boolean enableSpindexerManager = true;

    boolean overrideTurr = false;

    int intakeTicker = 0;
    private boolean shootAll = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.light.setPosition(0);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = new Servos(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        drive = new MecanumDrive(hardwareMap, teleStart);
        spindexer = new Spindexer(hardwareMap);
        targeting = new Targeting();
        targetingSettings = new Targeting.Settings(0.0, 0.0);

        drivetrain = new Drivetrain(robot, drive);

        PIDFController tController = new PIDFController(tp, ti, td, tf);

        tController.setTolerance(0.001);

        Turret turret = new Turret(robot, TELE, robot.limelight);
        robot.light.setPosition(1);
        while (opModeInInit()) {
            robot.limelight.start();
            if (redAlliance) {
                robot.limelight.pipelineSwitch(4);
            } else {
                robot.limelight.pipelineSwitch(2);
            }
        }

        limelightUsed= true;


        waitForStart();
        if (isStopRequested()) return;

        robot.transferServo.setPosition(transferServo_out);

        while (opModeIsActive()) {

            //TELE.addData("Is limelight on?", robot.limelight.getStatus());

            // LIGHT COLORS
            spindexer.ballCounterLight();

            //DRIVETRAIN:

            drivetrain.drive(
                    -gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x,
                    gamepad1.left_trigger
            );

            if (gamepad1.right_bumper) {
                shootAll = false;
                robot.transferServo.setPosition(transferServo_out);

            }

            robot.transfer.setPower(1);

            //TURRET TRACKING

            double robX = drive.localizer.getPose().position.x;
            double robY = drive.localizer.getPose().position.y;

            double robotX = robX - xOffset;
            double robotY = robY - yOffset;
            double robotHeading = drive.localizer.getPose().heading.toDouble();

            double goalX = -15;
            double goalY = 0;

            double dx = robotX - goalX;  // delta x from robot to goal
            double dy = robotY - goalY;  // delta y from robot to goal
            Pose2d deltaPose = new Pose2d(dx, dy, robotHeading);

            double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

            targetingSettings = targeting.calculateSettings
                    (robotX, robotY, robotHeading, 0.0, turretInterpolate);

            turret.trackGoal(deltaPose);

            //VELOCITY AUTOMATIC
            if (autoVel) {
                vel = targetingSettings.flywheelRPM;
            } else {
                vel = manualVel;
            }

            if (gamepad2.right_stick_button) {
                autoVel = true;
            } else if (gamepad2.right_stick_y < -0.5) {
                autoVel = false;
                manualVel = 4600;
            } else if (gamepad2.right_stick_y > 0.5) {
                autoVel = false;
                manualVel = 2700;
            } else if (gamepad2.right_stick_x > 0.5) {
                autoVel = false;
                manualVel = 3600;
            } else if (gamepad2.right_stick_x < -0.5) {
                autoVel = false;
                manualVel = 3100;
            }

            //SHOOTER:
            flywheel.manageFlywheel(vel);

            //HOOD:

            if (targetingHood) {
                robot.hood.setPosition(targetingSettings.hoodAngle + autoHoodOffset);
            } else {
                robot.hood.setPosition(hoodDefaultPos);
            }

            if (gamepad2.dpadUpWasPressed()) {
                autoHoodOffset -= hoodSpeedOffset;
                gamepad2.rumble(80);

            } else if (gamepad2.dpadDownWasPressed()) {
                autoHoodOffset += hoodSpeedOffset;
                gamepad2.rumble(80);
            }

            if (gamepad2.dpadLeftWasPressed()) {
                Turret.manualOffset -= turretSpeedOffset;
                gamepad2.rumble(80);
            } else if (gamepad2.dpadRightWasPressed()) {
                Turret.manualOffset += turretSpeedOffset;
                gamepad2.rumble(80);
            }

            if (gamepad2.rightBumperWasPressed()) {
                limelightUsed = true;
                gamepad2.rumble(80);
            } else if (gamepad2.leftBumperWasPressed()) {
                limelightUsed = false;
                gamepad2.rumble(80);
            }

            if (gamepad2.crossWasPressed()) {
                drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            }



            if (enableSpindexerManager) {
                //if (!shootAll) {
                spindexer.processIntake();
                //}

                // RIGHT_BUMPER
                if (gamepad1.right_bumper && intakeTicker > resetSpinTicks) {
                    robot.intake.setPower(1);
                } else if (gamepad1.cross) {
                    robot.intake.setPower(-1);

                } else {
                    robot.intake.setPower(0);

                }

                // LEFT_BUMPER
                if (!shootAll && gamepad1.leftBumperWasReleased()) {
                    shootStamp = getRuntime();
                    shootAll = true;

                    shooterTicker = 0;

                }
                intakeTicker++;
                if (shootAll) {
                    intakeTicker = 0;
                    intake = false;
                    reject = false;

                    if (shooterTicker == 0) {
                        spindexer.prepareShootAllContinous();
                        //TELE.addLine("preparing to shoot");
//                    } else if (shooterTicker == 2) {
//                        //robot.transferServo.setPosition(transferServo_in);
//                        spindexer.shootAll();
//                        TELE.addLine("starting to shoot");
                    } else if (!spindexer.shootAllComplete()) {
                        robot.transferServo.setPosition(transferServo_in);
                        //TELE.addLine("shoot");
                    } else {
                        robot.transferServo.setPosition(transferServo_out);
                        //spindexPos = spindexer_intakePos1;
                        shootAll = false;
                        spindexer.resetSpindexer();
                        //spindexer.processIntake();
                        //TELE.addLine("stop shooting");
                    }
                    shooterTicker++;
                    //spindexer.processIntake();
                }

                if (gamepad1.left_stick_button) {
                    robot.transferServo.setPosition(transferServo_out);
                    //spindexPos = spindexer_intakePos1;
                    shootAll = false;
                    spindexer.resetSpindexer();
                }
            }

            //EXTRA STUFFINESS:
            drive.updatePoseEstimate();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
//
//            TELE.addData("Spin1Green", green1 + ": " + ballIn(1));
//            TELE.addData("Spin2Green", green2 + ": " + ballIn(2));
//            TELE.addData("Spin3Green", green3 + ": " + ballIn(3));
//
//            TELE.addData("pose", drive.localizer.getPose());
//            TELE.addData("heading", drive.localizer.getPose().heading.toDouble());
//            TELE.addData("distanceToGoal", distanceToGoal);
//            TELE.addData("hood", robot.hood.getPosition());
//            TELE.addData("targetVel", vel);
//            TELE.addData("Velocity", flywheel.getVelo());
//            TELE.addData("Velo1", flywheel.velo1);
//            TELE.addData("Velo2", flywheel.velo2);
//            TELE.addData("shootOrder", shootOrder);
//            TELE.addData("oddColor", oddBallColor);
//
//            // Spindexer Debug
//            TELE.addData("spinEqual", servo.spinEqual(spindexer_intakePos1));
//            TELE.addData("spinCommmandedPos", spindexer.commandedIntakePosition);
//            TELE.addData("spinIntakeState", spindexer.currentIntakeState);
//            TELE.addData("spinTestCounter", spindexer.counter);
//            TELE.addData("autoSpintake", autoSpintake);
//
//            TELE.addData("shootall commanded", shootAll);
            // Targeting Debug
//            TELE.addData("robotX", robotX);
//            TELE.addData("robotY", robotY);
//            TELE.addData("robotInchesX", targeting.robotInchesX);
//            TELE.addData( "robotInchesY", targeting.robotInchesY);
//            TELE.addData("Targeting Interpolate", turretInterpolate);
//            TELE.addData("Targeting GridX", targeting.robotGridX);
//            TELE.addData("Targeting GridY", targeting.robotGridY);
//            TELE.addData("Targeting FlyWheel", targetingSettings.flywheelRPM);
//            TELE.addData("Targeting HoodAngle", targetingSettings.hoodAngle);
//            TELE.addData("timeSinceStamp", getRuntime() - shootStamp);
            TELE.addData("Voltage", robot.voltage.getVoltage());

            TELE.update();

            ticker++;
        }
    }

}
