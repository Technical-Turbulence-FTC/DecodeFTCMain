package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Poses.goalPose;
import static org.firstinspires.ftc.teamcode.constants.Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.tests.ShooterTest.kP;
import static org.firstinspires.ftc.teamcode.tests.ShooterTest.maxStep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class TeleopV2 extends LinearOpMode {

    public static double vel = 3000;
    public static double hood = 0.5;
    Robot robot;
    MultipleTelemetry TELE;
    boolean intake = false;
    boolean reject = false;
    int ticker = 0;
    List<Double> s1G = new ArrayList<>();
    List<Double> s2G = new ArrayList<>();
    List<Double> s3G = new ArrayList<>();
    List<Boolean> s1 = new ArrayList<>();
    List<Boolean> s2 = new ArrayList<>();
    List<Boolean> s3 = new ArrayList<>();
    public static double desiredTurretAngle = 180;
    MecanumDrive drive;
    private double lastEncoderRevolutions = 0.0;
    private double lastTimeStamp = 0.0;
    private double velo1, velo;
    private double stamp1, stamp, initPos;
    private boolean shootAll = false;

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

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            //DRIVETRAIN:

            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backRight.setPower(backRightPower);

            //INTAKE:

            if (gamepad1.rightBumperWasPressed()) {
                intake = true;
            }

            if (intake) {
                robot.intake.setPower(1);

                double position;

                if ((getRuntime() % 0.3) > 0.15) {
                    position = spindexer_intakePos1 + 0.015;
                } else {
                    position = spindexer_intakePos1 - 0.015;
                }

                robot.spin1.setPosition(position);
                robot.spin2.setPosition(1 - position);

            } else if (reject) {
                robot.intake.setPower(-1);
                double position = spindexer_intakePos1;
                robot.spin1.setPosition(position);
                robot.spin2.setPosition(1 - position);
            } else {
                robot.intake.setPower(0);
            }

            //COLOR:

            double s1D = robot.color1.getDistance(DistanceUnit.MM);
            double s2D = robot.color2.getDistance(DistanceUnit.MM);
            double s3D = robot.color3.getDistance(DistanceUnit.MM);

            if (s1D < 40) {

                double green = robot.color1.getNormalizedColors().green;
                double red = robot.color1.getNormalizedColors().red;
                double blue = robot.color1.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                s1G.add(gP);

                if (gP >= 0.43) {
                    s1.add(true);
                } else {
                    s1.add(false);
                }
            }

            if (s2D < 40) {

                double green = robot.color2.getNormalizedColors().green;
                double red = robot.color2.getNormalizedColors().red;
                double blue = robot.color2.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                s2G.add(gP);

                if (gP >= 0.43) {
                    s2.add(true);
                } else {
                    s2.add(false);
                }
            }

            if (s3D < 30) {

                double green = robot.color3.getNormalizedColors().green;
                double red = robot.color3.getNormalizedColors().red;
                double blue = robot.color3.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                s3G.add(gP);

                if (gP >= 0.43) {
                    s3.add(true);
                } else {
                    s3.add(false);
                }
            }

            boolean green1 = false;
            boolean green2 = false;
            boolean green3 = false;

            if (!s1.isEmpty()) {
                green1 = s1.get(s1.size() - 1);
            }
            if (!s2.isEmpty()) {
                green2 = s2.get(s2.size() - 1);
            }
            if (!s3.isEmpty()) {
                green3 = s3.get(s3.size() - 1);
            }

            //SHOOTER:

            double penguin = 0;

            if (ticker % 8 == 0) {
                penguin = (double) robot.shooterEncoder.getCurrentPosition() / 2048;
                double stamp = getRuntime();
                velo1 = -60 * ((penguin - initPos) / (stamp - stamp1));
                initPos = penguin;
                stamp1 = stamp;
            }

            velo = velo1;

            double feed = vel / 4500;

            if (vel > 500) {
                feed = Math.log((668.39 / (vel + 591.96)) - 0.116) / -4.18;
            }

            // --- PROPORTIONAL CORRECTION ---
            double error = vel - velo1;
            double correction = kP * error;

            // limit how fast power changes (prevents oscillation)
            correction = Math.max(-maxStep, Math.min(maxStep, correction));

            // --- FINAL MOTOR POWER ---
            double powPID = feed + correction;

            // clamp to allowed range
            powPID = Math.max(0, Math.min(1, powPID));

            if (vel - velo > 1000) {
                powPID = 1;
            } else if (velo - vel > 1000) {
                powPID = 0;
            }

            robot.shooter1.setPower(powPID);
            robot.shooter2.setPower(powPID);

            robot.hood.setPosition(hood);

            //TODO: ADD CODE TO CHANGE VARIABLE VEL BASED ON POSITION

            //TODO: ADD CODE TO CHANGE VARIABLE HOOD ANGLE BASED ON POSITION

            //SHOOT ALL:

            if (gamepad2.rightBumperWasPressed()) {
                shootAll = true;
            }

            if (shootAll) {
                intake = false;
                reject = false;

            }

            //TURRET:

            double offset;

            double robotX = drive.localizer.getPose().position.x;
            double robotY = drive.localizer.getPose().position.y;

            double goalX = -10;
            double goalY = 0;

            double dx = goalX - robotX;  // delta x from robot to goal
            double dy = goalY - robotY;  // delta y from robot to goal

            double angleRad = Math.atan2(dy, dx);


            desiredTurretAngle = (Math.toDegrees(Math.atan2(dy, dx)) + 360) % 360;

            offset = desiredTurretAngle - Math.toDegrees(drive.localizer.getPose().heading.toDouble());

            if (offset > 90) {
                offset -= 360;
            }

            double pos = 0.3;

            pos -= offset * (0.9 / 360);

            if (pos < 0.02) {
                pos = 0.02;
            } else if (pos > 0.91) {
                pos = 0.91;
            }

            robot.turr1.setPosition(pos);
            robot.turr2.setPosition(1 - pos);

            //MISC:

            drive.updatePoseEstimate();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            TELE.addData("Spin1Green", green1);
            TELE.addData("Spin2Green", green2);
            TELE.addData("Spin3Green", green3);

            TELE.addData("pose", drive.localizer.getPose());
            TELE.addData("heading", drive.localizer.getPose().heading.toDouble());

            TELE.update();

            ticker++;

        }
    }
}
