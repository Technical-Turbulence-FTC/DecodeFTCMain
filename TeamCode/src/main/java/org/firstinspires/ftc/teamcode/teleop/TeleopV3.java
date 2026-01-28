package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Servos.spinD;
import static org.firstinspires.ftc.teamcode.utils.Servos.spinF;
import static org.firstinspires.ftc.teamcode.utils.Servos.spinI;
import static org.firstinspires.ftc.teamcode.utils.Servos.spinP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class TeleopV3 extends LinearOpMode {
    public static double manualVel = 3000;
    public static int intakeJamSwap = 12;
    public static double hoodDefaultPos = 0.5;
    public static double desiredTurretAngle = 180;
    public static double shootStamp2 = 0.0;
    public static double spinningPow = 0.2;
    public static double spindexPos = spindexer_intakePos1;
    public static double spinPow = 0.09;
    public static double bMult = 1, bDiv = 2200;
    public static double tp = 0.8, ti = 0.001, td = 0.0315, tf = 0;
    public static boolean manualTurret = true;
    public double vel = 3000;
    public boolean autoVel = true;
    public boolean targetingVel = true;
    public boolean targetingHood = true;
    public double manualOffset = 0.0;
    public boolean autoHood = true;
    public boolean green1 = false;
    public boolean green2 = false;
    public boolean green3 = false;
    public double shootStamp = 0.0;
    public boolean circle = false;
    public boolean square = false;
    public boolean triangle = false;
    public TranslationalVelConstraint VEL_CONSTRAINT = new TranslationalVelConstraint(200);
    public ProfileAccelConstraint ACCEL_CONSTRAINT = new ProfileAccelConstraint(-Math.abs(60), 200);
    boolean fixedTurret = false;
    PIDFController spinPID = new PIDFController(spinP, spinI, spinD, spinF);
    Robot robot;
    MultipleTelemetry TELE;
    Servos servo;
    Flywheel flywheel;
    MecanumDrive drive;
    Spindexer spindexer;
    Targeting targeting;
    Targeting.Settings targetingSettings;
    double autoHoodOffset = 0.0;

    int shooterTicker = 0;
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
    double hoodOffset = 0.0;
    boolean shootA = true;
    boolean shootB = true;
    boolean shootC = true;
    boolean autoSpintake = false;
    boolean enableSpindexerManager = true;
    List<Integer> shootOrder = new ArrayList<>();
    boolean outtake1 = false;
    boolean outtake2 = false;
    boolean outtake3 = false;
    boolean overrideTurr = false;
    double turretPID = 0.0;
    double turretPos = 0;
    double spindexPID = 0.0;
    double error = 0.0;
    double spinCurrentPos = 0.0, spinInitPos = 0.0, intakeStamp = 0.0;
    boolean reverse = false;
    int intakeTicker = 0;
    Pose2d brakePos = new Pose2d(0, 0, 0);
    boolean autoDrive = false;
    private boolean shootAll = false;
    private double transferStamp = 0.0;
    private int tickerA = 1;
    private boolean transferIn = false;
    boolean turretInterpolate = false;
    public static double spinSpeedIncrease = 0.03;
    public static int resetSpinTicks = 4;

    public static double velPrediction(double distance) {
        if (distance < 30) {
            return 2750;
        } else if (distance > 100) {
            if (distance > 120) {
                return 4500;
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

        PIDFController tController = new PIDFController(tp, ti, td, tf);

        tController.setTolerance(0.001);
//
//        if (redAlliance) {
//            robot.limelight.pipelineSwitch(3);
//        } else {
//            robot.limelight.pipelineSwitch(2);
//        }

//        robot.limelight.start();

        Turret turret = new Turret(robot, TELE, robot.limelight);
        robot.light.setPosition(1);
        waitForStart();
        if (isStopRequested()) return;

        robot.transferServo.setPosition(transferServo_out);

        while (opModeIsActive()) {
            // LIGHT COLORS
            spindexer.ballCounterLight();

            //DRIVETRAIN:

            double y = 0.0;
            double x = 0.0;
            double rx = 0.0;
            if (!autoDrive) {

                y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
                x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
                rx = gamepad1.left_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                robot.frontLeft.setPower(frontLeftPower);
                robot.backLeft.setPower(backLeftPower);
                robot.frontRight.setPower(frontRightPower);
                robot.backRight.setPower(backRightPower);
            }

            if (gamepad1.left_trigger > 0.4 && robot.frontLeft.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE && !autoDrive) {
                robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                drive.updatePoseEstimate();

                brakePos = drive.localizer.getPose();
                autoDrive = true;

            } else if (gamepad1.left_trigger > 0.4) {
                drive.updatePoseEstimate();

                Pose2d currentPos = drive.localizer.getPose();

                TrajectoryActionBuilder traj2 = drive.actionBuilder(currentPos)
                        .strafeToLinearHeading(new Vector2d(brakePos.position.x, brakePos.position.y), brakePos.heading.toDouble(), VEL_CONSTRAINT, ACCEL_CONSTRAINT);

                if (Math.abs(currentPos.position.x - brakePos.position.x) > 1 || Math.abs(currentPos.position.y - brakePos.position.y) > 1) {
                    Actions.runBlocking(
                            traj2.build()
                    );
                }
            } else {
                autoDrive = false;
                robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }

            //TODO: Use color sensors to switch between positions...switch after ball detected in

            if (gamepad1.right_bumper){
                shootAll = false;
                robot.transferServo.setPosition(transferServo_out);

            }

            if (autoSpintake) {

                if (!servo.spinEqual(spindexPos) && !gamepad1.right_bumper) {


                    robot.spin1.setPosition(spindexPos);
                    robot.spin2.setPosition(1-spindexPos);

                }

                if (gamepad1.right_bumper) {

                    shootAll = false;

                    intakeTicker++;

//                    if (intakeTicker % 20 < 2) {
//
//                        robot.spin1.setPower(-1);
//                        robot.spin2.setPower(1);
//
//                    } else if (intakeTicker % 20 < 10) {
//                        robot.spin1.setPower(-0.5);
//                        robot.spin2.setPower(0.5);
//                    } else if (intakeTicker % 20 < 12) {
//                        robot.spin1.setPower(1);
//                        robot.spin2.setPower(-1);
//                    } else {
//                        robot.spin1.setPower(0.5);
//                        robot.spin2.setPower(-0.5);
//                    }

                    robot.intake.setPower(1);
                    intakeStamp = getRuntime();
                    TELE.addData("Reverse?", reverse);
                    TELE.update();
                } else {
                    if (!servo.spinEqual(spindexPos)) {
                        robot.spin1.setPosition(spindexPos);
                        robot.spin2.setPosition(1-spindexPos);

                    }

                    spindexPos = spindexer_intakePos1;

                    robot.intake.setPower(0);

                    intakeTicker = 0;
                }
            }

            //COLOR:

            double s1D = robot.color1.getDistance(DistanceUnit.MM);
            double s2D = robot.color2.getDistance(DistanceUnit.MM);
            double s3D = robot.color3.getDistance(DistanceUnit.MM);

            if (s1D < 43) {

                double green = robot.color1.getNormalizedColors().green;
                double red = robot.color1.getNormalizedColors().red;
                double blue = robot.color1.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                s1G.add(gP);
                TELE.addData("gp1", gP);

                if (gP >= 0.36) {
                    s1.add(true);
                } else {
                    s1.add(false);
                }

                s1T.add(getRuntime());

            }

            if (s2D < 60) {

                double green = robot.color2.getNormalizedColors().green;
                double red = robot.color2.getNormalizedColors().red;
                double blue = robot.color2.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                s2G.add(gP);
                TELE.addData("gp2", gP);

                if (gP >= 0.43) {
                    s2.add(true);
                } else {
                    s2.add(false);
                }

                s2T.add(getRuntime());
            }

            if (s3D < 33) {

                double green = robot.color3.getNormalizedColors().green;
                double red = robot.color3.getNormalizedColors().red;
                double blue = robot.color3.getNormalizedColors().blue;

                double gP = green / (green + red + blue);

                TELE.addData("gp3", gP);

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

            robot.transfer.setPower(1);

            double offset;

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
                    (robotX,robotY,robotHeading,0.0, turretInterpolate);

            turret.trackGoal(deltaPose);

            //VELOCITY AUTOMATIC
            if (targetingVel) {
                vel = targetingSettings.flywheelRPM;
            } else if (autoVel) {
                vel = velPrediction(distanceToGoal);
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

            //TODO: test the camera teleop code

//            if (y < 0.3 && y > -0.3 && x < 0.3 && x > -0.3 && rx < 0.3 && rx > -0.3) { //not moving
//                double bearing;
//
//                LLResult result = robot.light.getLatestResult();
//                if (result != null) {
//                    if (result.isValid()) {
//                        bearing = result.getTx() * bMult;
//
//                        double bearingCorrection = bearing / bDiv;
//
//                        error += bearingCorrection;
//
//                        camTicker++;
//                        TELE.addData("tx", bearingCorrection);
//                        TELE.addData("ty", result.getTy());
//                    }
//                }
//
//            } else {
//                camTicker = 0;
//                overrideTurr = false;
//            }

            //HOOD:

            if (targetingHood) {
                robot.hood.setPosition(targetingSettings.hoodAngle);
            } else if (autoHood) {
                robot.hood.setPosition(0.15 + hoodOffset);
            } else {
                robot.hood.setPosition(hoodDefaultPos + hoodOffset);
            }

            if (gamepad2.dpadUpWasPressed() || gamepad1.dpadUpWasPressed()) {
                hoodOffset -= 0.03;
                autoHoodOffset -= 0.02;

            } else if (gamepad2.dpadDownWasPressed() || gamepad1.dpadDownWasPressed()) {
                hoodOffset += 0.03;
                autoHoodOffset += 0.02;

            }
//

            if (gamepad2.cross) {
                manualOffset = 0;
                overrideTurr = true;
            }

            if (gamepad2.squareWasPressed()) {
                drive = new MecanumDrive(hardwareMap, new Pose2d(20, 0, 0));
                sleep(1500);
            }

            if (gamepad2.triangle) {
                autoHood = false;
                hoodOffset = 0;
            }

            if (gamepad2.circleWasPressed()) {
                xOffset = robotX;
                yOffset = robotY;
                headingOffset = robotHeading;

                autoHood = true;
                fixedTurret = false;
            }
//
//            if (gamepad2.left_stick_y < -0.5) {
//                autoHood = true;
//            } else if (gamepad2.left_stick_y > 0.5) {
//                autoHood = false;
//                hoodOffset = 0;
//                if (gamepad2.left_bumper) {
//                    xOffset = robotX;
//                    yOffset = robotY;
//                    headingOffset = robotHeading;
//                }
//            }

            //if (gamepad1.left_bumper && !enableSpindexerManager) {

            //    robot.transferServo.setPosition(transferServo_out);

//                autoSpintake = false;
//
//                intakeTicker++;
//
//                if (intakeTicker % 10 < 1) {
//
//                    robot.spin1.setPower(-1);
//                    robot.spin2.setPower(1);
//
//                } else if (intakeTicker % 10 < 5) {
//                    robot.spin1.setPower(-0.5);
//                    robot.spin2.setPower(0.5);
//                } else if (intakeTicker % 10 < 6) {
//                    robot.spin1.setPower(1);
//                    robot.spin2.setPower(-1);
//                } else {
//                    robot.spin1.setPower(0.5);
//                    robot.spin2.setPower(-0.5);
//                }
//
//                intake = false;
//                reject = false;
//
//                robot.intake.setPower(0.5);
//
//            }
//
//            if (gamepad1.leftBumperWasReleased() && !enableSpindexerManager) {
//                shootStamp = getRuntime();
//                shootAll = true;
//
//                shooterTicker = 0;
//            }
//
//            if (shootAll && !enableSpindexerManager) {
//
//                TELE.addData("100% works", shootOrder);
//
//                intake = false;
//                reject = false;
//
//                shooterTicker++;
//
//                spindexPos = spindexer_intakePos1;
//
//                if (getRuntime() - shootStamp < 3.5) {
//
//                    robot.transferServo.setPosition(transferServo_in);
//
//                    autoSpintake = false;
//
//                    robot.spin1.setPower(-spinPow);
//                    robot.spin2.setPower(spinPow);
//
//                } else {
//                    robot.transferServo.setPosition(transferServo_out);
//                    spindexPos = spindexer_intakePos1;
//
//                    shootAll = false;
//
//                    autoSpintake = true;
//
//                    robot.transferServo.setPosition(transferServo_out);
//                }
//
//            }

            if (enableSpindexerManager) {
                spindexer.processIntake();

                // RIGHT_BUMPER
                if (gamepad1.right_bumper && intakeTicker > resetSpinTicks) {
                    robot.intake.setPower(1);
                } else {
                    robot.intake.setPower(0);

                }

                // LEFT_BUMPER
                if (!shootAll &&
                        (gamepad1.leftBumperWasReleased() ||
                                gamepad1.leftBumperWasPressed() ||
                                gamepad1.left_bumper)) {
                    shootStamp = getRuntime();
                    shootAll = true;

                    shooterTicker = 0;

                }
                intakeTicker++;
                if (shootAll) {
                    intakeTicker = 0;
                    intake = false;
                    reject = false;

                    if (servo.getSpinPos() < spindexer_outtakeBall2 + 0.4) {

                        if (shooterTicker == 0){
                            robot.transferServo.setPosition(transferServo_out);
                            robot.spin1.setPosition(spinStartPos);
                            robot.spin2.setPosition(1-spinStartPos);
                            if (servo.spinEqual(spinStartPos)){
                                shooterTicker++;
                            }
                            TELE.addLine("starting to shoot");
                        } else {
                            robot.transferServo.setPosition(transferServo_in);
                            shooterTicker++;
                            double prevSpinPos = servo.getSpinPos();
                            robot.spin1.setPosition(prevSpinPos + spinSpeedIncrease);
                            robot.spin2.setPosition(1 - prevSpinPos - spinSpeedIncrease);
                            TELE.addLine("shooting");
                        }

                    } else {
                        robot.transferServo.setPosition(transferServo_out);
                        //spindexPos = spindexer_intakePos1;

                        shootAll = false;

                        spindexer.resetSpindexer();
                        spindexer.processIntake();
                        TELE.addLine("stop shooting");
                    }
                }

                if (gamepad1.left_stick_button){
                    robot.transferServo.setPosition(transferServo_out);
                    //spindexPos = spindexer_intakePos1;

                    shootAll = false;
                    spindexer.resetSpindexer();
                    spindexer.processIntake();
                }
            }

//
//            if (shootAll) {
//
//                TELE.addData("100% works", shootOrder);
//
//                intake = false;
//                reject = false;
//
//                shooterTicker++;
//
//                spindexPos = spindexer_intakePos1;
//
//                if (getRuntime() - shootStamp < 1) {
//
//                    if (servo.spinEqual(spindexer_outtakeBall3) || ((getRuntime()-shootStamp)>0.4)){
//                        robot.transferServo.setPosition(transferServo_in);
//
//                    } else {
//                        robot.transferServo.setPosition(transferServo_out);
//
//                    }
//
//
//                    autoSpintake = true;
//
//                    spindexPos = spindexer_outtakeBall3;
//                    robot.intake.setPower(0.5);
//
//                }
//
//                else if (getRuntime() - shootStamp < 1.8) {
//
//                    robot.transferServo.setPosition(transferServo_in);
//
//                    autoSpintake = true;
//                    robot.intake.setPower(0);
//
//                    spindexPos = spindexer_outtakeBall2;
//
//                }
//                else if (getRuntime() - shootStamp < 2.6) {
//
//                    robot.transferServo.setPosition(transferServo_in);
//
//                    autoSpintake = false;
//
//                    robot.spin1.setPower(1);
//                    robot.spin2.setPower(-1);
//
//                }
//
//                else {
//                    robot.transferServo.setPosition(transferServo_out);
//                    spindexPos = spindexer_intakePos1;
//
//                    shootAll = false;
//
//                    autoSpintake = true;
//
//                    robot.transferServo.setPosition(transferServo_out);
//                }
//
//            }

//                if (gamepad1.squareWasPressed()) {
//                    square = true;
//                    shootStamp = getRuntime();
//                    shootStamp2 = getRuntime();
//                    outtake1 = false;
//                    outtake2 = false;
//                    outtake3 = false;
//                }
//
//                if (gamepad1.circleWasPressed()) {
//                    circle = true;
//                    shootStamp = getRuntime();
//                    shootStamp2 = getRuntime();
//
//                    outtake1 = false;
//                    outtake2 = false;
//                    outtake3 = false;
//
//                }
//
//                if (gamepad1.triangleWasPressed()) {
//                    triangle = true;
//                    shootStamp = getRuntime();
//                    shootStamp2 = getRuntime();
//
//                    outtake1 = false;
//                    outtake2 = false;
//                    outtake3 = false;
//
//                }
//
//                if (square || circle || triangle) {
//
//                    // Count green balls
//                    int greenCount = 0;
//                    if (green1) greenCount++;
//                    if (green2) greenCount++;
//                    if (green3) greenCount++;
//
//                    // Determine the odd ball color
//                    oddBallColor = greenCount < 2; // true = green, false = purple
//
//                    shootOrder.clear();
//
//                    // Determine shooting order based on button pressed
//                    // square = odd ball first, triangle = odd ball second, circle = odd ball third
//                    if (square) {
//                        // Odd ball first
//                        addOddThenRest(shootOrder, oddBallColor);
//
//                    } else if (triangle) {
//                        // Odd ball second
//                        addOddInMiddle(shootOrder, oddBallColor);
//                    } else if (circle) {
//                        // Odd ball last
//                        addOddLast(shootOrder, oddBallColor);
//                    }
//
//                    circle = false;
//                    square = false;
//                    triangle = false;
//
//                }
//
//                // Right bumper shoots all balls fastest, ignoring colors
//                if (gamepad1.crossWasPressed()) {
//                    shootOrder.clear();
//                    shootStamp = getRuntime();
//
//                    outtake1 = false;
//                    outtake2 = false;
//                    outtake3 = false;
//
//                    // Fastest order (example: slot 3 → 2 → 1)
//                    if (ballIn(3)) {
//                        shootOrder.add(3);
//                    }
//
//                    if (ballIn(2)) {
//                        shootOrder.add(2);
//                    }
//
//                    if (ballIn(1)) {
//                        shootOrder.add(1);
//                    }
//
//                    if (!shootOrder.contains(3)) {
//                        shootOrder.add(3);
//                    }
//
//                    if (!shootOrder.contains(2)) {
//                        shootOrder.add(2);
//                    }
//
//                    if (!shootOrder.contains(1)) {
//                        shootOrder.add(1);
//                    }
//
//                    shootAll = true;
//                }

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
//            // Targeting Debug
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

            TELE.update();

            ticker++;
        }
    }

    // Helper methods
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

    //
//    public boolean shootTeleop(double spindexer, boolean spinOk, double stamp) {
//        // Set spin positions
//        spindexPos = spindexer;
//
//        // Check if spindexer has reached the target position
//        if (spinOk || getRuntime() - stamp > 1.5) {
//            if (tickerA == 1) {
//                transferStamp = getRuntime();
//                tickerA++;
//                TELE.addLine("tickerSet");
//            }
//
//            if (getRuntime() - transferStamp > waitTransfer && !transferIn) {
//                robot.transferServo.setPosition(transferServo_in);
//                transferIn = true;
//                TELE.addLine("transferring");
//
//                return true; // still in progress
//
//            } else if (getRuntime() - transferStamp > waitTransfer + waitTransferOut && transferIn) {
//                robot.transferServo.setPosition(transferServo_out);
//                transferIn = false; // reset for next shot
//                tickerA = 1;        // reset ticker
//                transferStamp = 0.0;
//
//                TELE.addLine("shotFinished");
//
//                return false;       // finished shooting
//            } else {
//                TELE.addLine("sIP");
//                return true; // still in progress
//            }
//        } else {
//            robot.transferServo.setPosition(transferServo_out);
//            tickerA = 1;
//            transferStamp = getRuntime();
//            transferIn = false;
//            return true; // still moving spin
//        }
//    }
//
    public double hoodAnglePrediction(double x) {
        double a = 1.44304;
        double b = 0.0313707;
        double c = 0.0931136;

        double result = a * Math.exp(-b * x) + c;

        // Clamp between min and max
        if (result < 0.1) {
            return 0.1;
        } else if (result > 0.96) {
            return 0.96;
        } else {
            return result;
        }
    }

    //
//    void addOddThenRest(List<Integer> order, boolean oddColor) {
//        // Odd ball first
//        for (int i = 1; i <= 3; i++) if (getBallColor(i) == oddColor) order.add(i);
//        TELE.addData("1", shootOrder);
//        for (int i = 1; i <= 3; i++) if (getBallColor(i) != oddColor) order.add(i);
//        TELE.addData("works", shootOrder);
//        TELE.addData("oddBall", oddColor);
//        shootAll = true;
//
//    }
//
//    void addOddInMiddle(List<Integer> order, boolean oddColor) {
//
//        boolean[] used = new boolean[4];   // index 1..3
//
//        // 1) Add a NON-odd ball first
//        for (int i = 1; i <= 3; i++) {
//            if (getBallColor(i) != oddColor) {
//                order.add(i);
//                used[i] = true;
//                break;
//            }
//        }
//
//        // 2) Add the odd ball second
//        for (int i = 1; i <= 3; i++) {
//            if (!used[i] && getBallColor(i) == oddColor) {
//                order.add(i);
//                used[i] = true;
//                break;
//            }
//        }
//
//        // 3) Add the remaining non-odd ball third
//        for (int i = 1; i <= 3; i++) {
//            if (!used[i] && getBallColor(i) != oddColor) {
//                order.add(i);
//                used[i] = true;
//                break;
//            }
//        }
//
//        TELE.addData("works", order);
//        TELE.addData("oddBall", oddColor);
//        shootAll = true;
//
//    }
//
//    void addOddLast(List<Integer> order, boolean oddColor) {
//        // Odd ball last
//        for (int i = 1; i <= 3; i++) if (getBallColor(i) != oddColor) order.add(i);
//        TELE.addData("1", shootOrder);
//        for (int i = 1; i <= 3; i++) if (getBallColor(i) == oddColor) order.add(i);
//        TELE.addData("works", shootOrder);
//        TELE.addData("oddBall", oddColor);
//        shootAll = true;
//
//    }
//
//    // Returns color of ball in slot i (1-based)
//    boolean getBallColor(int slot) {
//        switch (slot) {
//            case 1:
//                return green1;
//            case 2:
//                return green2;
//            case 3:
//                return green3;
//        }
//        return false; // default
//    }
//
    boolean ballIn(int slot) {
        List<Double> times =
                slot == 1 ? s1T :
                        slot == 2 ? s2T :
                                slot == 3 ? s3T : null;

        if (times == null || times.isEmpty()) return false;

        return times.get(times.size() - 1) > getRuntime() - 2;
    }
}
