package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.tests.ShooterTest.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.ArrayList;
import java.util.List;

public class TeleopV2 extends LinearOpMode {

    Robot robot;
    MultipleTelemetry TELE;

    boolean intake = false;
    boolean reject = false;
    private double lastEncoderRevolutions = 0.0;
    private double lastTimeStamp = 0.0;

    private double vel = 3000;

    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    List<Double> s1G = new ArrayList<>();
    List<Double> s2G = new ArrayList<>();
    List<Double> s3G = new ArrayList<>();
    List<Boolean> s1 = new ArrayList<>();
    List<Boolean> s2 = new ArrayList<>();
    List<Boolean> s3 = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

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
                }
            }

            boolean green1 = s1.get(s1.size() - 1);
            boolean green2 = s2.get(s2.size() - 1);
            boolean green3 = s3.get(s3.size() - 1);

        //SHOOTER:

            double kF = 1.0 / MAX_RPM;     // baseline feedforward

            double encoderRevolutions = (double) robot.shooterEncoder.getCurrentPosition() / 2048;

            double velocity = -60 * (encoderRevolutions - lastEncoderRevolutions) / (getRuntime() - lastTimeStamp);

            double velPID;

            // --- FEEDFORWARD BASE POWER ---
            double feed = kF * vel;        // Example: vel=2500 → feed=0.5

            // --- PROPORTIONAL CORRECTION ---
            double error = vel - velocity;
            double correction = kP * error;

            // limit how fast power changes (prevents oscillation)
            correction = Math.max(-maxStep, Math.min(maxStep, correction));

            // --- FINAL MOTOR POWER ---
            velPID = feed + correction;

            // clamp to allowed range
            velPID = Math.max(0, Math.min(1, velPID));

            robot.shooter1.setPower(velPID);
            robot.shooter2.setPower(velPID);

            //TODO: ADD CODE TO CHANGE VARIABLE VEL BASED ON POSITION

        //MISC:

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            TELE.addData("Spin1Green", s1.get(s1.size() - 1));
            TELE.addData("Spin2Green", s2.get(s2.size() - 1));
            TELE.addData("Spin3Green", s3.get(s3.size() - 1));

            TELE.update();

        }
    }
}
