package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bHGate;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bShoot1Tangent;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bShootH;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bShootX;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bShootY;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bXGate;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bYGate;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bh1;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bh2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bh2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bh2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bt2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bt2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bt2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bx1;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bx2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bx2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.bx2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.by1;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.by2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.by2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.by2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rHGate;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rShoot1Tangent;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rShootH;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rShootX;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rShootY;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rXGate;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rYGate;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rh1;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rh2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rh2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rh2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rt2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rt2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rt2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rx1;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rx2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rx2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.rx2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.ry1;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.ry2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.ry2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.ry2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_LT_Indexed.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.util.ArrayList;
import java.util.List;

@Autonomous(preselectTeleOp = "TeleopV3")
@Config
public class Auto_LT_Indexed extends LinearOpMode {

    public static double shoot0Vel = 2300, shoot0Hood = 0.93;
    public static double autoSpinStartPos = 0.2;
    public static double shoot0SpinSpeedIncrease = 0.014;

    public static double spindexerSpeedIncrease = 0.02;

    public static double obeliskTurrPos = 0.53;
    public static double gatePickupTime = 3.0;
    public static double shoot1Turr = 0.57;
    public static double shoot0XTolerance = 1.0;
    public static double turretShootPos = 0.72;
    public static double shootAllTime = 1.8;
    public static double shoot0Time = 1.6;
    public static double intake1Time = 3.0;
    public static double flywheel0Time = 3.5;
    public static double pickup1Speed = 20.0;
    public static double pickup2Speed = 20.0;
    public static double pickup3Speed = 20.0;

    // ---- SECOND SHOT / PICKUP ----
    public static double shoot1Vel = 2300;
    public static double shoot1Hood = 0.93;

    public static double shootAllVelocity = 2500;
    public static double shootAllHood = 0.78;
    // ---- PICKUP TIMING ----
    public static double pickup1Time = 3.0;
    // ---- PICKUP POSITION TOLERANCES ----
    public static double pickup1XTolerance = 2.0;
    public static double pickup1YTolerance = 2.0;
    // ---- OBELISK DETECTION ----
    public static double obelisk1Time = 1.5;
    public static double obelisk1XTolerance = 2.0;
    public static double obelisk1YTolerance = 2.0;
    public static double shoot1ToleranceX = 2.0;
    public static double shoot1ToleranceY = 2.0;
    public static double shoot1Time = 2.0;
    public static double shootCycleTime = 2.5;

    List<Boolean> s1 = new ArrayList<>();

    List<Boolean> s2 = new ArrayList<>();
    List<Boolean> s3 = new ArrayList<>();
    public int motif = 0;
    Robot robot;
    MultipleTelemetry TELE;
    MecanumDrive drive;
    Servos servos;
    Spindexer spindexer;
    Flywheel flywheel;
    Turret turret;
    Targeting targeting;
    Targeting.Settings targetingSettings;
    private double x1, y1, h1;

    private double x2a, y2a, h2a, t2a;

    private double x2b, y2b, h2b, t2b;
    private double x2c, y2c, h2c, t2c;

    private double xShoot, yShoot, hShoot;
    private double xGate, yGate, hGate;

    private double shoot1Tangent;

    // ---- OPEN GATE / MIDFIELD ----
    private double x3, y3, h3, t3;

    // ---- PICKUP 2 ----
    private double x4a, y4a, h4a;
    private double x4b, y4b, h4b;

    // ---- PICKUP 3 ----
    private double x5a, y5a, h5a;
    private double x5b, y5b, h5b;

    // ---- PARK ----
    private double xPark, yPark, hPark;

    public Action prepareShootIndexed(double time) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            boolean testColor = false;

            int s1Green = 0;
            int s2Green = 0;
            int s3Green = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;

                if ((System.currentTimeMillis() - stamp) < (goToDetectTime)) { //0.25-0.4 ish???
                    robot.spin1.setPosition(colorDetectPos); //0.43
                    robot.spin2.setPosition(1 - colorDetectPos);
                } else if ((System.currentTimeMillis() - stamp) < (colorSenseTime + goToDetectTime)) {


                    double green1 = robot.color1.getNormalizedColors().green;
                    double red1 = robot.color1.getNormalizedColors().red;
                    double blue1 = robot.color1.getNormalizedColors().blue;

                    double gP1 = green1 / (green1 + red1 + blue1);


                    if (gP1 >= color1Thresh) {
                        s1Green ++;
                    }

                    double green2 = robot.color2.getNormalizedColors().green;
                    double red2 = robot.color2.getNormalizedColors().red;
                    double blue2 = robot.color2.getNormalizedColors().blue;

                    double gP2 = green2 / (green2 + red2 + blue2);

                    if (gP2 >= color2Thresh) {
                        s2Green ++;
                    }


                    double green3 = robot.color3.getNormalizedColors().green;
                    double red3 = robot.color3.getNormalizedColors().red;
                    double blue3 = robot.color3.getNormalizedColors().blue;

                    double gP3 = green3 / (green3 + red3 + blue3);

                    if (gP3 >= color3Thresh) {
                        s3Green ++;
                    }


                    testColor = true;



                } else {

                    double spindexPos;
                    if (motif == 21) {
                        spindexPos =
                    }
                }

                robot.transferServo.setPosition(transferServo_out);

                turret.manualSetTurret(turretShootPos);

                robot.intake.setPower(-((System.currentTimeMillis() - stamp)) / 1000);

                return (System.currentTimeMillis() - stamp) < (time * 1000);

            }
        };
    }

    public Action shootAll(int vel, double shootTime, double spindexSpeed) {
        return new Action() {
            int ticker = 1;

            double stamp = 0.0;

            double velo = vel;

            int shooterTicker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                TELE.addData("Velocity", velo);
                TELE.addLine("shooting");
                TELE.update();

                flywheel.manageFlywheel(vel);
                velo = flywheel.getVelo();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                robot.intake.setPower(-0.3);

                if (ticker == 1) {
                    stamp = getRuntime();
                }
                ticker++;

                robot.intake.setPower(0);

                if (getRuntime() - stamp < shootTime) {

                    if (shooterTicker == 0 && !servos.spinEqual(autoSpinStartPos)) {
                        robot.spin1.setPosition(autoSpinStartPos);
                        robot.spin2.setPosition(1 - autoSpinStartPos);
                    } else {
                        robot.transferServo.setPosition(transferServo_in);
                        shooterTicker++;
                        double prevSpinPos = robot.spin1.getPosition();
                        robot.spin1.setPosition(prevSpinPos + spindexSpeed);
                        robot.spin2.setPosition(1 - prevSpinPos - spindexSpeed);
                    }

                    return true;

                } else {
                    robot.transferServo.setPosition(transferServo_out);
                    //spindexPos = spindexer_intakePos1;

                    spindexer.resetSpindexer();
                    spindexer.processIntake();

                    return false;

                }

            }
        };
    }

    public Action shootAllAuto(double shootTime, double spindexSpeed) {
        return new Action() {
            int ticker = 1;

            double stamp = 0.0;

            double velo = 0.0;

            int shooterTicker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                TELE.addData("Velocity", velo);
                TELE.addLine("shooting");
                TELE.update();

                velo = flywheel.getVelo();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                robot.intake.setPower(-0.3);

                if (ticker == 1) {
                    stamp = getRuntime();
                }
                ticker++;

                robot.intake.setPower(0);

                if (getRuntime() - stamp < shootTime) {

                    if (shooterTicker == 0 && !servos.spinEqual(autoSpinStartPos)) {
                        robot.spin1.setPosition(autoSpinStartPos);
                        robot.spin2.setPosition(1 - autoSpinStartPos);
                    } else {
                        robot.transferServo.setPosition(transferServo_in);
                        shooterTicker++;
                        double prevSpinPos = robot.spin1.getPosition();
                        robot.spin1.setPosition(prevSpinPos + spindexSpeed);
                        robot.spin2.setPosition(1 - prevSpinPos - spindexSpeed);
                    }

                    return true;

                } else {
                    robot.transferServo.setPosition(transferServo_out);
                    //spindexPos = spindexer_intakePos1;

                    spindexer.resetSpindexer();
                    spindexer.processIntake();

                    return false;

                }

            }
        };
    }

    public Action intake(double intakeTime) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;

                spindexer.processIntake();
                robot.intake.setPower(1);

                motif = turret.detectObelisk();

                spindexer.ballCounterLight();

                return (System.currentTimeMillis() - stamp) < (intakeTime * 1000);

            }
        };
    }

    public Action detectObelisk(
            double time,
            double posX,
            double posY,
            double posXTolerance,
            double posYTolerance,
            double turrPos
    ) {

        boolean timeFallback = (time != 0.501);
        boolean posXFallback = (posX != 0.501);
        boolean posYFallback = (posY != 0.501);

        return new Action() {

            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }

                ticker++;

                motif = turret.detectObelisk();

                robot.turr1.setPosition(turrPos);
                robot.turr2.setPosition(1 - turrPos);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || xDone || yDone;

                return !shouldFinish;

            }
        };
    }

    public Action manageFlywheel(
            double vel,
            double hoodPos,
            double time,
            double posX,
            double posY,
            double posXTolerance,
            double posYTolerance
    ) {

        boolean timeFallback = (time != 0.501);
        boolean posXFallback = (posX != 0.501);
        boolean posYFallback = (posY != 0.501);

        return new Action() {

            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }

                ticker++;

                flywheel.manageFlywheel(vel);
                robot.hood.setPosition(hoodPos);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || xDone || yDone;

                return !shouldFinish;

            }
        };
    }

    public Action manageShooterAuto(
            double time,
            double posX,
            double posY,
            double posXTolerance,
            double posYTolerance
    ) {

        boolean timeFallback = (time != 0.501);
        boolean posXFallback = (posX != 0.501);
        boolean posYFallback = (posY != 0.501);

        return new Action() {

            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }

                ticker++;

                double robotX = drive.localizer.getPose().position.x;
                double robotY = drive.localizer.getPose().position.y;

                double robotHeading = drive.localizer.getPose().heading.toDouble();

                double goalX = -15;
                double goalY = 0;

                double dx = robotX - goalX;  // delta x from robot to goal
                double dy = robotY - goalY;  // delta y from robot to goal
                Pose2d deltaPose = new Pose2d(dx, dy, robotHeading);

                double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

                targetingSettings = targeting.calculateSettings
                        (robotX, robotY, robotHeading, 0.0, false);

                turret.trackGoal(deltaPose);

                robot.hood.setPosition(targetingSettings.hoodAngle);

                flywheel.manageFlywheel(targetingSettings.flywheelRPM);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || xDone || yDone;

                return !shouldFinish;

            }
        };
    }

    public Action manageFlywheelAuto(
            double time,
            double posX,
            double posY,
            double posXTolerance,
            double posYTolerance
    ) {

        boolean timeFallback = (time != 0.501);
        boolean posXFallback = (posX != 0.501);
        boolean posYFallback = (posY != 0.501);

        return new Action() {

            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }

                ticker++;

                double robotX = drive.localizer.getPose().position.x;
                double robotY = drive.localizer.getPose().position.y;

                double robotHeading = drive.localizer.getPose().heading.toDouble();

                double goalX = -15;
                double goalY = 0;

                double dx = robotX - goalX;  // delta x from robot to goal
                double dy = robotY - goalY;  // delta y from robot to goal
                Pose2d deltaPose = new Pose2d(dx, dy, robotHeading);

                double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

                targetingSettings = targeting.calculateSettings
                        (robotX, robotY, robotHeading, 0.0, false);

                robot.hood.setPosition(targetingSettings.hoodAngle);

                flywheel.manageFlywheel(targetingSettings.flywheelRPM);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || xDone || yDone;

                return !shouldFinish;

            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap.get(Servo.class, "light").setPosition(0);

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        flywheel = new Flywheel(hardwareMap);

        targeting = new Targeting();
        targetingSettings = new Targeting.Settings(0.0, 0.0);

        spindexer = new Spindexer(hardwareMap);

        servos = new Servos(hardwareMap);

        robot.limelight.pipelineSwitch(1);

        turret = new Turret(robot, TELE, robot.limelight);

        turret.manualSetTurret(0.4);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        TrajectoryActionBuilder shoot0 = null;

        TrajectoryActionBuilder pickup1 = null;
        TrajectoryActionBuilder shoot1 = null;
        TrajectoryActionBuilder pickup2 = null;
        TrajectoryActionBuilder shoot2 = null;
        TrajectoryActionBuilder pickup3 = null;
        TrajectoryActionBuilder shoot3 = null;
        TrajectoryActionBuilder openGate = null;
        TrajectoryActionBuilder park = null;

        robot.spin1.setPosition(autoSpinStartPos);
        robot.spin2.setPosition(1 - autoSpinStartPos);

        robot.transferServo.setPosition(transferServo_out);

        robot.light.setPosition(1);

        while (opModeInInit()) {

            robot.hood.setPosition(shoot0Hood);

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;
            }

            if (redAlliance) {
                robot.light.setPosition(0.28);

                // ===== FIRST SHOT =====
                x1 = rx1;
                y1 = ry1;
                h1 = rh1;

                // ===== PICKUP PATH =====
                x2a = rx2a;
                y2a = ry2a;
                h2a = rh2a;
                t2a = rt2a;

                x2b = rx2b;
                y2b = ry2b;
                h2b = rh2b;
                t2b = rt2b;

                x2c = rx2c;
                y2c = ry2c;
                h2c = rh2c;
                t2c = rt2c;

                // ===== SHOOT POSE =====
                xShoot = rShootX;
                yShoot = rShootY;
                hShoot = rShootH;
                shoot1Tangent = rShoot1Tangent;

                // ===== GATE =====
                xGate = rXGate;
                yGate = rYGate;
                hGate = rHGate;

            } else {
                robot.light.setPosition(0.6);

                // ===== FIRST SHOT =====
                x1 = bx1;
                y1 = by1;
                h1 = bh1;

                // ===== PICKUP PATH =====
                x2a = bx2a;
                y2a = by2a;
                h2a = bh2a;
                t2a = bt2a;

                x2b = bx2b;
                y2b = by2b;
                h2b = bh2b;
                t2b = bt2b;

                x2c = bx2c;
                y2c = by2c;
                h2c = bh2c;
                t2c = bt2c;

                // ===== SHOOT POSE =====
                xShoot = bShootX;
                yShoot = bShootY;
                hShoot = bShootH;
                shoot1Tangent = bShoot1Tangent;

                // ===== GATE =====
                xGate = bXGate;
                yGate = bYGate;
                hGate = bHGate;
            }

            shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .strafeToLinearHeading(new Vector2d(x1, y1), h1);

            pickup1 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                    .strafeToLinearHeading(new Vector2d(x2a, y2a), h2a)
                    .strafeToLinearHeading(new Vector2d(x2b, y2b), h2b,
                            new TranslationalVelConstraint(pickup1Speed));

            openGate = drive.actionBuilder(new Pose2d(x2b, y2b, h2b))
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(x3, y3), t3);

            shoot1 = drive.actionBuilder(new Pose2d(x3, y3, h2b))
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), hShoot);

            pickup2 = drive.actionBuilder(new Pose2d(xShoot, yShoot, hShoot))
                    .strafeToLinearHeading(new Vector2d(x4a, y4a), h4a)
                    .strafeToLinearHeading(new Vector2d(x4b, y4b), h4b,
                            new TranslationalVelConstraint(pickup2Speed));
            shoot2 = drive.actionBuilder(new Pose2d(x4b, y4b, h4b))
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), hShoot);

            pickup3 = drive.actionBuilder(new Pose2d(xShoot, yShoot, hShoot))
                    .strafeToLinearHeading(new Vector2d(x5a, y5a), h5a)
                    .strafeToLinearHeading(new Vector2d(x5b, y5b), h5b,
                            new TranslationalVelConstraint(pickup3Speed));
            shoot3 = drive.actionBuilder(new Pose2d(x5b, y5b, h5b))
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), hShoot);
            park = drive.actionBuilder(new Pose2d(xShoot, yShoot, hShoot))
                    .strafeToLinearHeading(new Vector2d(xPark, yPark), hPark);

        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            robot.transfer.setPower(1);

            assert shoot0 != null;

            Actions.runBlocking(
                    new ParallelAction(
                            shoot0.build(),
                            manageFlywheel(
                                    shoot0Vel,
                                    shoot0Hood,
                                    flywheel0Time,
                                    x1,
                                    0.501,
                                    shoot0XTolerance,
                                    0.501
                            )

                    )
            );

            Actions.runBlocking(
                    shootAll((int) shoot0Vel, shoot0Time, shoot0SpinSpeedIncrease)
            );

            //WORKING SHOOT ALL for the preload

            //Pickup first pile

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    pickup1.build(),
                                    manageFlywheel(
                                            shootAllVelocity,
                                            shootAllHood,
                                            pickup1Time,
                                            x2b,
                                            y2b,
                                            pickup1XTolerance,
                                            pickup1YTolerance
                                    ),
                                    intake(intake1Time),
                                    detectObelisk(
                                            obelisk1Time,
                                            x2b,
                                            y2c,
                                            obelisk1XTolerance,
                                            obelisk1YTolerance,
                                            obeliskTurrPos
                                    )
                            ),
                            new ParallelAction(
                                    openGate.build() //TODO: Add other managing stuff here
                            )
                    )

            );

            motif = turret.detectObelisk(); //Detect Obelisk if not alr

            Actions.runBlocking(
                    new ParallelAction(
                            manageFlywheel(
                                    shootAllVelocity,
                                    shootAllHood,
                                    shoot1Time,
                                    0.501,
                                    0.501,
                                    0.501,
                                    0.501
                            ),
                            shoot1.build(),
                            prepareShootIndexed(shoot1Time)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            manageShooterAuto(
                                    shootAllTime,
                                    0.501,
                                    0.501,
                                    0.501,
                                    0.501
                            ),
                            shootAllIndexed(shootAllTime, spindexerSpeedIncrease)
                    )

            );

            //Shoot from first pile

        }

    }
}
