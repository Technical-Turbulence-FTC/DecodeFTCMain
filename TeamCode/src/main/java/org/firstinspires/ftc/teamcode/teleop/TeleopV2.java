package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Poses.teleStart;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.utils.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.HoodSubsystem;
import org.firstinspires.ftc.teamcode.utils.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.utils.TurretSubsystem;

import java.util.List;

@TeleOp
@Config
public class TeleopV2 extends LinearOpMode {

    Robot robot;
    MultipleTelemetry TELE;
    MecanumDrive drive;
    
    // Subsystems
    Drivetrain drivetrain;
    IntakeSubsystem intakeSubsystem;
    ColorSensorSubsystem colorSensor;
    ShooterSubsystem shooter;
    TurretSubsystem turret;
    HoodSubsystem hood;
    SpindexerSubsystem spindexer;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    
    // State variables
    double xOffset = 0.0;
    double yOffset = 0.0;
    double headingOffset = 0.0;
    boolean emergency = false;
    boolean circle = false;
    boolean square = false;
    boolean triangle = false;
    int ticker = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        drive = new MecanumDrive(hardwareMap, teleStart);

        // Initialize subsystems
        drivetrain = new Drivetrain(robot);
        intakeSubsystem = new IntakeSubsystem(robot);
        colorSensor = new ColorSensorSubsystem(robot);
        shooter = new ShooterSubsystem(robot, TELE);
        turret = new TurretSubsystem(robot, TELE);
        hood = new HoodSubsystem(robot);
        spindexer = new SpindexerSubsystem(robot, TELE, colorSensor, turret);
        aprilTagWebcam.init(new Robot(hardwareMap), TELE);

        robot.turr1.setPosition(0.4);
        robot.turr2.setPosition(1 - 0.4);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            //DRIVETRAIN:

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x * 1.1;
            double rx = gamepad1.left_stick_x;

            drivetrain.setDrivePower(y, x, rx);

            //INTAKE:

            if (gamepad1.rightBumperWasPressed()) {
                intakeSubsystem.toggleIntake();
                spindexer.stopShooting();
                emergency = false;
                turret.setOverride(false);
            }

            if (gamepad1.leftBumperWasPressed()) {
                intakeSubsystem.setIntake(false);
                emergency = !emergency;
            }

            intakeSubsystem.update(getRuntime());

            if (emergency) {
                intakeSubsystem.setReject(true);
            }

            //COLOR:

            colorSensor.update(getRuntime());

            //SHOOTER:

            double robX = drive.localizer.getPose().position.x;
            double robY = drive.localizer.getPose().position.y;
            double robotX = robX - xOffset;
            double robotY = robY - yOffset;

            double goalX = -10;
            double goalY = 0;
            double dx = goalX - robotX;
            double dy = goalY - robotY;
            double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

            shooter.update(getRuntime(), distanceToGoal);
            shooter.handleVelocityInput(gamepad2.right_stick_button, gamepad2.right_stick_y, gamepad2.right_stick_x);

            //TURRET:

            double robotHeading = drive.localizer.getPose().heading.toDouble();
            turret.updateAutoAim(robotX, robotY, robotHeading - headingOffset);

            if (gamepad2.dpad_right) {
                turret.adjustManualOffset(-2);
            } else if (gamepad2.dpad_left) {
                turret.adjustManualOffset(2);
            }



            //HOOD:

            hood.update(distanceToGoal);

            if (gamepad2.dpadUpWasPressed()) {
                hood.adjustOffset(-0.03);
                hood.adjustAutoOffset(-0.02);
            } else if (gamepad2.dpadDownWasPressed()) {
                hood.adjustOffset(0.03);
                hood.adjustAutoOffset(0.02);
            }

            if (gamepad2.left_stick_x > 0.5) {
                turret.setManualMode(false);
            } else if (gamepad2.left_stick_x < -0.5) {
                turret.resetManualOffset();
                turret.setManualMode(false);
                if (gamepad2.left_bumper) {
                    drive = new MecanumDrive(hardwareMap, new Pose2d(2, 0, 0));
                    sleep(1200);
                }
            }

            if (gamepad2.left_stick_y < -0.5) {
                hood.setAutoMode(true);
            } else if (gamepad2.left_stick_y > 0.5) {
                hood.setAutoMode(false);
                if (gamepad2.left_bumper) {
                    xOffset = robotX;
                    yOffset = robotY;
                    headingOffset = robotHeading;
                }
            }

            //SHOOT ALL:

            if (emergency) {
                intakeSubsystem.setIntake(false);
                intakeSubsystem.setReject(true);
                spindexer.updateEmergency(getRuntime());
            } else {
                spindexer.update(getRuntime(), aprilTagWebcam);
                
                // Update turret override based on spindexer state
                if (spindexer.isShootingAll()) {
                    turret.setOverride(true);
                }
            }

            if (gamepad2.squareWasPressed()) {
                square = true;
            }

            if (gamepad2.circleWasPressed()) {
                circle = true;
            }

            if (gamepad2.triangleWasPressed()) {
                triangle = true;
            }

            if (square || circle || triangle) {
                if (square) {
                    spindexer.startShootOddFirst(getRuntime());
                } else if (triangle) {
                    spindexer.startShootOddMiddle(getRuntime());
                } else if (circle) {
                    spindexer.startShootOddLast(getRuntime());
                }

                circle = false;
                square = false;
                triangle = false;
            }

            if (gamepad2.rightBumperWasPressed()) {
                spindexer.startShootFastest(getRuntime());
            }
            if (gamepad2.crossWasPressed()) {
                emergency = true;

            }

            if (gamepad2.leftBumperWasPressed()) {
                emergency = false;
            }

            //MISC:

            drive.updatePoseEstimate();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            TELE.addData("Spin1Green", colorSensor.isGreen1() + ": " + colorSensor.ballIn(1, getRuntime()));
            TELE.addData("Spin2Green", colorSensor.isGreen2() + ": " + colorSensor.ballIn(2, getRuntime()));
            TELE.addData("Spin3Green", colorSensor.isGreen3() + ": " + colorSensor.ballIn(3, getRuntime()));

            TELE.addData("pose", drive.localizer.getPose());
            TELE.addData("heading", drive.localizer.getPose().heading.toDouble());
            TELE.addData("distanceToGoal", distanceToGoal);
            TELE.addData("hood", hood.getCurrentPosition());

            TELE.addData("shootOrder", spindexer.getShootOrder());

            aprilTagWebcam.update();

            TELE.update();

            ticker++;

        }
    }
}
