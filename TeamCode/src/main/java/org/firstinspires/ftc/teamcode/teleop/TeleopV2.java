package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.*;
import static org.firstinspires.ftc.teamcode.tests.PIDServoTest.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp
@Config
public class TeleopV2 extends LinearOpMode {
    Servos servo;
    Flywheel flywheel;
    public static double manualVel = 3000;
    public static double hood = 0.5;
    public static double hoodDefaultPos = 0.5;
    public static double desiredTurretAngle = 180;
    public static double velMultiplier = 20;
    public static double shootStamp2 = 0.0;

    public double vel = 3000;
    public boolean autoVel = true;
    public double manualOffset = 0.0;
    public boolean autoHood = true;
    public boolean green1 = false;
    public boolean green2 = false;
    public boolean green3 = false;
    public double shootStamp = 0.0;
    public boolean circle = false;
    public boolean square = false;
    public boolean triangle = false;
    double autoHoodOffset = 0.0;
    Robot robot;
    MultipleTelemetry TELE;
    boolean intake = false;
    boolean reject = false;
    double xOffset = 0.0;
    double yOffset = 0.0;
    double headingOffset = 0.0;
    int ticker = 0;
    int camTicker = 0;
    List<Double> s1G = new ArrayList<>();
    List<Double> s2G = new ArrayList<>();
    List<Double> s3G = new ArrayList<>();
    List<Double> s1T = new ArrayList<>();
    List<Double> s2T = new ArrayList<>();
    List<Double> s3T = new ArrayList<>();
    List<Boolean> s1 = new ArrayList<>();
    List<Boolean> s2 = new ArrayList<>();
    List<Boolean> s3 = new ArrayList<>();
    boolean oddBallColor = false;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    MecanumDrive drive;
    double hoodOffset = 0.0;
    boolean shoot1 = false;
    // Make these class-level flags
    boolean shootA = true;
    boolean shootB = true;
    boolean shootC = true;
    boolean manualTurret = false;

    boolean outtake1 = false;
    boolean outtake2 = false;
    boolean outtake3 = false;
    boolean overrideTurr = false;

    List<Integer> shootOrder = new ArrayList<>();
    boolean emergency = false;
    private double lastEncoderRevolutions = 0.0;
    private double lastTimeStamp = 0.0;
    private double velo1, velo;
    private double stamp1, stamp, initPos;
    private boolean shootAll = false;
    private double transferStamp = 0.0;
    private int tickerA = 1;
    private boolean transferIn = false;
    double turretPID = 0.0;
    double turretPos = 0.5;
    double spindexPID = 0.0;
    double spindexPos = spindexer_intakePos1;
    double error = 0.0;

    public static double velPrediction(double distance) {

        if (distance < 30) {
            return 2750;
        } else if (distance > 100) {
            if (distance > 160) {
                return 4200;
            }
            return 3700;
        } else {
            // linear interpolation between 40->2650 and 120->3600
            double slope = (3700.0 - 2750.0) / (100.0 - 30);
            return (int) Math.round(2750 + slope * (distance - 30));
        }

    }

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
        servo = new Servos(hardwareMap);
        flywheel = new Flywheel(hardwareMap);

        drive = new MecanumDrive(hardwareMap, teleStart);

        Pose2d shootPos = teleStart;

        aprilTagWebcam.init(new Robot(hardwareMap), TELE);

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


            //TODO: make sure changing position works throughout opmode
            if (!servo.spinEqual(spindexPos)){
                spindexPID = servo.setSpinPos(spindexPos);
                robot.spin1.setPosition(spindexPID);
                robot.spin2.setPosition(-spindexPID);
            } else{
                robot.spin1.setPosition(0);
                robot.spin2.setPosition(0);
            }

            //INTAKE:

            if (gamepad1.rightBumperWasPressed()) {
                intake = !intake;
                reject = false;
                shootAll = false;
                emergency = false;
                overrideTurr = false;

            }

            if (gamepad1.leftBumperWasPressed()) {
                intake = false;
                emergency = !emergency;

            }

            if (intake) {

                robot.transferServo.setPosition(transferServo_out);

                robot.intake.setPower(1);

                if ((getRuntime() % 0.3) > 0.15) {
                    spindexPos = spindexer_intakePos1 + 0.015;
                } else {
                    spindexPos = spindexer_intakePos1 - 0.015;
                }

            } else if (reject) {
                robot.intake.setPower(-1);
                spindexPos = spindexer_intakePos1;
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

                s1T.add(getRuntime());

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

                s2T.add(getRuntime());
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

                s3T.add(getRuntime());
            }

            if (!s1.isEmpty()) {
                green1 = checkGreen(s1, s1T);
            }
            if (!s2.isEmpty()) {
                green2 = checkGreen(s2, s2T);

            }
            if (!s3.isEmpty()) {
                green3 = checkGreen(s3, s3T);
            }

            //SHOOTER:

            double powPID = flywheel.manageFlywheel((int) vel);

             robot.transfer.setPower(1);

            //TURRET:

            double offset;

            double robX = drive.localizer.getPose().position.x;
            double robY = drive.localizer.getPose().position.y;

            double robotX = robX - xOffset;
            double robotY = robY - yOffset;
            double robotHeading = drive.localizer.getPose().heading.toDouble();

            double goalX = -10;
            double goalY = 0;

            double dx = goalX - robotX;  // delta x from robot to goal
            double dy = goalY - robotY;  // delta y from robot to goal

            double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

            desiredTurretAngle = (Math.toDegrees(Math.atan2(dy, dx)) + 360) % 360;

            desiredTurretAngle += manualOffset;

            offset = desiredTurretAngle - 180 - (Math.toDegrees(robotHeading - headingOffset));

            if (offset > 135) {
                offset -= 360;
            }

            //TODO: test the camera teleop code
            double pos = turrDefault + (error/8); // adds the overall error to the default

            TELE.addData("offset", offset);

            pos -= offset * (0.9 / 360);

            if (pos < 0.02) {
                pos = 0.02;
            } else if (pos > 0.97) {
                pos = 0.97;
            }

            if (y < 0.1 && y > -0.1 && x < 0.1 && x > -0.1 && rx < 0.1 && rx > -0.1){ //not moving
                AprilTagDetection d20 = aprilTagWebcam.getTagById(20);
                AprilTagDetection d24 = aprilTagWebcam.getTagById(24);

                double bearing = 0.0;
                if (d20 != null || d24 != null){
                    if (d20 != null) {
                        bearing = d20.ftcPose.bearing;
                    }
                    if (d24 != null) {
                        bearing = d24.ftcPose.bearing;
                    }
                    overrideTurr = true;
                    turretPos = servo.getTurrPos() - (bearing/1300);
                    TELE.addData("Bear", bearing);

                    double bearingCorrection = bearing / 1300;


                    // deadband: ignore tiny noise
                    if (Math.abs(bearing) > 0.3 && camTicker < 8) {

                        // only accumulate if bearing direction is consistent
                        if (Math.signum(bearingCorrection) == Math.signum(error) || error == 0) {
                            error += bearingCorrection;
                        }
                    }

                    camTicker++;

                }


            } else {
                camTicker = 0;
                overrideTurr = false;

            }

            if (manualTurret) {
                pos = turrDefault + (manualOffset / 100);
            }

            if (!overrideTurr) {
                turretPos = pos;
            }

            if (gamepad2.dpad_right) {
                manualOffset -= 2;
            } else if (gamepad2.dpad_left) {
                manualOffset += 2;
            }

            //VELOCITY AUTOMATIC

            if (autoVel) {
                vel = velPrediction(distanceToGoal);
            } else {
                vel = manualVel;
            }

            if (gamepad2.right_stick_button) {
                autoVel = true;
            } else if (gamepad2.right_stick_y < -0.5) {
                autoVel = false;
                manualVel = 4100;
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

            //HOOD:

            if (autoHood) {
                robot.hood.setPosition(hoodAnglePrediction(distanceToGoal) + autoHoodOffset);
            } else {
                robot.hood.setPosition(hoodDefaultPos + hoodOffset);
            }

            if (gamepad2.dpadUpWasPressed()) {
                hoodOffset -= 0.03;
                autoHoodOffset -= 0.02;

            } else if (gamepad2.dpadDownWasPressed()) {
                hoodOffset += 0.03;
                autoHoodOffset += 0.02;

            }

            if (gamepad2.left_stick_x > 0.5) {
                manualTurret = false;
            } else if (gamepad2.left_stick_x < -0.5) {
                manualOffset = 0;
                manualTurret = false;
                if (gamepad2.left_bumper) {
                    drive = new MecanumDrive(hardwareMap, new Pose2d(2, 0, 0));
                    sleep(1200);
                }
            }

            if (gamepad2.left_stick_y < -0.5) {
                autoHood = true;
            } else if (gamepad2.left_stick_y > 0.5) {
                autoHood = false;
                hoodOffset = 0;
                if (gamepad2.left_bumper) {
                    xOffset = robotX;
                    yOffset = robotY;
                    headingOffset = robotHeading;
                }
            }

            //SHOOT ALL:]

            if (emergency) {
                intake = false;
                reject = true;

                if (getRuntime() % 3 > 1.5) {
                    spindexPos = 1;
                } else {
                    spindexPos = 0;
                }

                robot.transferServo.setPosition(transferServo_out);

                robot.transfer.setPower(1);

            } else if (shootAll) {

                TELE.addData("100% works", shootOrder);

                intake = false;
                reject = false;

                if (!shootOrder.isEmpty() && (getRuntime() - shootStamp < 12)) {
                    int currentSlot = shootOrder.get(0); // Peek, do NOT remove yet
                    boolean shootingDone = false;

                    if (!outtake1) {
                        outtake1 = (servo.spinEqual(spindexer_outtakeBall1));
                    }
                    if (!outtake2) {
                        outtake2 = (servo.spinEqual(spindexer_outtakeBall2));
                    }
                    if (!outtake3) {
                        outtake3 = (servo.spinEqual(spindexer_outtakeBall3));
                    }

                    switch (currentSlot) {
                        case 1:
                            shootA = shootTeleop(spindexer_outtakeBall1, outtake1, shootStamp2);
                            TELE.addData("shootA", shootA);

                            if ((getRuntime() - shootStamp) < 4 * (4 - shootOrder.size())) {
                                shootingDone = !shootA;
                            } else {
                                shootingDone = true;
                            }
                            break;
                        case 2:
                            shootB = shootTeleop(spindexer_outtakeBall2, outtake2, shootStamp2);
                            TELE.addData("shootB", shootB);
                            if ((getRuntime() - shootStamp) < 4 * (4 - shootOrder.size())) {
                                shootingDone = !shootB;
                            } else {
                                shootingDone = true;
                            }
                            break;
                        case 3:
                            shootC = shootTeleop(spindexer_outtakeBall3, outtake3, shootStamp2);
                            TELE.addData("shootC", shootC);
                            if ((getRuntime() - shootStamp) < 4 * (4 - shootOrder.size())) {
                                shootingDone = !shootC;
                            } else {
                                shootingDone = true;
                            }
                            break;
                    }

                    // Remove from the list only if shooting is complete
                    if (shootingDone) {
                        shootOrder.remove(0);
                        shootStamp2 = getRuntime();

                    }

                } else {
                    // Finished shooting all balls
                    spindexPos = spindexer_intakePos1;
                    shootA = true;
                    shootB = true;
                    shootC = true;
                    reject = false;
                    intake = true;
                    shootAll = false;
                    outtake1 = false;
                    outtake2 = false;
                    outtake3 = false;

                    overrideTurr = false;

                }

            }

            if (gamepad2.squareWasPressed()) {
                square = true;
                shootStamp = getRuntime();
                shootStamp2 = getRuntime();
                outtake1 = false;
                outtake2 = false;
                outtake3 = false;
            }

            if (gamepad2.circleWasPressed()) {
                circle = true;
                shootStamp = getRuntime();
                shootStamp2 = getRuntime();

                outtake1 = false;
                outtake2 = false;
                outtake3 = false;

            }

            if (gamepad2.triangleWasPressed()) {
                triangle = true;
                shootStamp = getRuntime();
                shootStamp2 = getRuntime();

                outtake1 = false;
                outtake2 = false;
                outtake3 = false;

            }

            if (square || circle || triangle) {

                // Count green balls
                int greenCount = 0;
                if (green1) greenCount++;
                if (green2) greenCount++;
                if (green3) greenCount++;

                // Determine the odd ball color
                oddBallColor = greenCount < 2; // true = green, false = purple

                shootOrder.clear();

                // Determine shooting order based on button pressed
                // square = odd ball first, triangle = odd ball second, circle = odd ball third
                if (square) {
                    // Odd ball first
                    addOddThenRest(shootOrder, oddBallColor);

                } else if (triangle) {
                    // Odd ball second
                    addOddInMiddle(shootOrder, oddBallColor);
                } else if (circle) {
                    // Odd ball last
                    addOddLast(shootOrder, oddBallColor);
                }

                circle = false;
                square = false;
                triangle = false;

            }

            // Right bumper shoots all balls fastest, ignoring colors
            if (gamepad2.rightBumperWasPressed()) {
                shootOrder.clear();
                shootStamp = getRuntime();

                outtake1 = false;
                outtake2 = false;
                outtake3 = false;

                // Fastest order (example: slot 3 → 2 → 1)

                if (ballIn(3)) {
                    shootOrder.add(3);

                }

                if (ballIn(2)) {
                    shootOrder.add(2);

                }

                if (ballIn(1)) {
                    shootOrder.add(1);

                }

                if (!shootOrder.contains(3)) {

                    shootOrder.add(3);
                }

                if (!shootOrder.contains(2)) {

                    shootOrder.add(2);
                }

                if (!shootOrder.contains(1)) {

                    shootOrder.add(1);
                }

                shootAll = true;
                shootPos = drive.localizer.getPose();

            }

//            // Right bumper shoots all balls fastest, ignoring colors
//            if (gamepad2.leftBumperWasPressed()) {
//                shootOrder.clear();
//                shootStamp = getRuntime();
//
//                outtake1 = false;
//                outtake2 = false;
//                outtake3 = false;
//
//                // Fastest order (example: slot 3 → 2 → 1)
//
//                if (ballIn(3)) {
//                    shootOrder.add(3);
//                }
//
//                if (ballIn(2)) {
//                    shootOrder.add(2);
//                }
//                if (ballIn(1)) {
//                    shootOrder.add(1);
//                }
//                shootAll = true;
//                shootPos = drive.localizer.getPose();
//
//            }
//
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

            TELE.addData("Spin1Green", green1 + ": " + ballIn(1));
            TELE.addData("Spin2Green", green2 + ": " + ballIn(2));
            TELE.addData("Spin3Green", green3 + ": " + ballIn(3));

            TELE.addData("pose", drive.localizer.getPose());
            TELE.addData("heading", drive.localizer.getPose().heading.toDouble());
            TELE.addData("distanceToGoal", distanceToGoal);
            TELE.addData("hood", robot.hood.getPosition());
            TELE.addData("targetVel", vel);

            TELE.addData("shootOrder", shootOrder);
            TELE.addData("oddColor", oddBallColor);

            aprilTagWebcam.update();

            TELE.update();

            ticker++;

        }
    }

    // Helper method
    private boolean checkGreen(List<Boolean> s, List<Double> sT) {
        if (s.isEmpty()) return false;

        double lastTime = sT.get(sT.size() - 1);
        int countTrue = 0;
        int countWindow = 0;

        for (int i = 0; i < s.size(); i++) {
            if (lastTime - sT.get(i) <= 3.0) {  // element is within 2s of last
                countWindow++;
                if (s.get(i)) {
                    countTrue++;
                }
            }
        }

        if (countWindow == 0) return false; // avoid divide by zero
        return countTrue > countWindow / 2.0; // more than 50% true
    }

    public boolean shootTeleop(double spindexer, boolean spinOk, double stamp) {
        // Set spin positions
        spindexPos = spindexer;

        // Check if spindexer has reached the target position
        if (spinOk || getRuntime() - stamp > 1.5) {
            if (tickerA == 1) {
                transferStamp = getRuntime();
                tickerA++;
                TELE.addLine("tickerSet");
            }

            if (getRuntime() - transferStamp > waitTransfer && !transferIn) {
                robot.transferServo.setPosition(transferServo_in);
                transferIn = true;
                TELE.addLine("transferring");

                return true; // still in progress

            } else if (getRuntime() - transferStamp > waitTransfer + waitTransferOut && transferIn) {
                robot.transferServo.setPosition(transferServo_out);
                transferIn = false; // reset for next shot
                tickerA = 1;        // reset ticker
                transferStamp = 0.0;

                TELE.addLine("shotFinished");

                return false;       // finished shooting
            } else {
                TELE.addLine("sIP");
                return true; // still in progress
            }
        } else {
            robot.transferServo.setPosition(transferServo_out);
            tickerA = 1;
            transferStamp = getRuntime();
            transferIn = false;
            return true; // still moving spin
        }
    }

    public double hoodAnglePrediction(double x) {
        if (x < 34) {
            double L = 1.04471;
            double U = 0.711929;
            double Q = 120.02263;
            double B = 0.780982;
            double M = 20.61191;
            double v = 10.40506;

            double inner = 1 + Q * Math.exp(-B * (x - M));
            return L + (U - L) / Math.pow(inner, 1.0 / v);

        } else {
            // x >= 34
            return 1.94372 * Math.exp(-0.0528731 * x) + 0.39;
        }
    }

    void addOddThenRest(List<Integer> order, boolean oddColor) {
        // Odd ball first
        for (int i = 1; i <= 3; i++) if (getBallColor(i) == oddColor) order.add(i);
        TELE.addData("1", shootOrder);
        for (int i = 1; i <= 3; i++) if (getBallColor(i) != oddColor) order.add(i);
        TELE.addData("works", shootOrder);
        TELE.addData("oddBall", oddColor);
        shootAll = true;

    }

    void addOddInMiddle(List<Integer> order, boolean oddColor) {

        boolean[] used = new boolean[4];   // index 1..3

        // 1) Add a NON-odd ball first
        for (int i = 1; i <= 3; i++) {
            if (getBallColor(i) != oddColor) {
                order.add(i);
                used[i] = true;
                break;
            }
        }

        // 2) Add the odd ball second
        for (int i = 1; i <= 3; i++) {
            if (!used[i] && getBallColor(i) == oddColor) {
                order.add(i);
                used[i] = true;
                break;
            }
        }

        // 3) Add the remaining non-odd ball third
        for (int i = 1; i <= 3; i++) {
            if (!used[i] && getBallColor(i) != oddColor) {
                order.add(i);
                used[i] = true;
                break;
            }
        }

        TELE.addData("works", order);
        TELE.addData("oddBall", oddColor);
        shootAll = true;

    }

    void addOddLast(List<Integer> order, boolean oddColor) {
        // Odd ball last
        for (int i = 1; i <= 3; i++) if (getBallColor(i) != oddColor) order.add(i);
        TELE.addData("1", shootOrder);
        for (int i = 1; i <= 3; i++) if (getBallColor(i) == oddColor) order.add(i);
        TELE.addData("works", shootOrder);
        TELE.addData("oddBall", oddColor);
        shootAll = true;

    }

    // Returns color of ball in slot i (1-based)
    boolean getBallColor(int slot) {
        switch (slot) {
            case 1:
                return green1;
            case 2:
                return green2;
            case 3:
                return green3;
        }
        return false; // default
    }

    boolean ballIn(int slot) {
        switch (slot) {
            case 1:

                if (!s1T.isEmpty()) {

                    return !(s1T.get(s1T.size() - 1) < (getRuntime()) - 3);
                }

            case 2:

                if (!s2T.isEmpty()) {

                    return !(s2T.get(s2T.size() - 1) < (getRuntime()) - 3);
                }

            case 3:

                if (!s3T.isEmpty()) {

                    return !(s3T.get(s3T.size() - 1) < (getRuntime()) - 3);

                }
        }
        return true; // default
    }
}
