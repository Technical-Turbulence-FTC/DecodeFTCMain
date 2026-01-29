package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Poses.bShootH;
import static org.firstinspires.ftc.teamcode.constants.Poses.bShootX;
import static org.firstinspires.ftc.teamcode.constants.Poses.bShootY;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh1;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bhPrep;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx1;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bxPrep;
import static org.firstinspires.ftc.teamcode.constants.Poses.by1;
import static org.firstinspires.ftc.teamcode.constants.Poses.by2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.by2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.by3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.by3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.by4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.by4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.byPrep;
import static org.firstinspires.ftc.teamcode.constants.Poses.rShootH;
import static org.firstinspires.ftc.teamcode.constants.Poses.rShootX;
import static org.firstinspires.ftc.teamcode.constants.Poses.rShootY;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh1;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rhPrep;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx1;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rxPrep;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry1;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.ryPrep;
import static org.firstinspires.ftc.teamcode.constants.Poses.teleStart;
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
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.Poses_V2;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class ProtoAutoClose_V4 extends LinearOpMode {
    public static double shoot0Vel = 2300, shoot0Hood = 0.93;
    public static double autoSpinStartPos = 0.2;
    public static double shoot0SpinSpeedIncrease = 0.014;

    public static double spindexerSpeedIncrease = 0.02;

    public static double obeliskTurrPos = 0.53;
    public static double normalIntakeTime = 3.0;
    public static double shoot1Turr = 0.57;
    public static double shoot0XTolerance = 1.0;
    public static double turretShootPos = 0.53;
    public static double shootAllTime = 1.8;
    public static double shoot0Time = 1.6;
    public static double intake1Time = 3.0;
    public static double flywheel0Time = 3.5;
    public static double pickup1Speed = 25;
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

    private double x3a, y3a, h3a;
    private double x3b, y3b, h3b;
    private double x4a, y4a, h4a;
    private double x4b, y4b, h4b;

    private double xShoot, yShoot, hShoot;
    private double xGate, yGate, hGate;
    private double xPrep, yPrep, hPrep;

    private double shoot1Tangent;


    public Action prepareShootAll(double time) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;

                robot.spin1.setPosition(autoSpinStartPos);
                robot.spin2.setPosition(1 - autoSpinStartPos);

                robot.transferServo.setPosition(transferServo_out);

                turret.manualSetTurret(turretShootPos);

                robot.intake.setPower(-((System.currentTimeMillis() - stamp)) / 1000);
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

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
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

                flywheel.manageFlywheel(vel);
                velo = flywheel.getVelo();

                drive.updatePoseEstimate();

                Poses_V2.teleStart = drive.localizer.getPose();

                robot.intake.setPower(-0.3);

                if (ticker == 1) {
                    stamp = getRuntime();
                }
                ticker++;

                robot.intake.setPower(0);
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

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
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

                velo = flywheel.getVelo();

                drive.updatePoseEstimate();

                Poses_V2.teleStart = drive.localizer.getPose();

                robot.intake.setPower(-0.3);

                if (ticker == 1) {
                    stamp = getRuntime();
                }
                ticker++;

                robot.intake.setPower(0);
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

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
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

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
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

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
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

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
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

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
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

                return !shouldFinish;

            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {

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

        robot.spin1.setPosition(autoSpinStartPos);
        robot.spin2.setPosition(1 - autoSpinStartPos);

        robot.transferServo.setPosition(transferServo_out);

        TrajectoryActionBuilder shoot0 = null;
        TrajectoryActionBuilder pickup1 = null;
        TrajectoryActionBuilder shoot1 = null;
        TrajectoryActionBuilder pickup2 = null;
        TrajectoryActionBuilder shoot2 = null;
        TrajectoryActionBuilder pickup3 = null;
        TrajectoryActionBuilder shoot3 = null;

        robot.limelight.start();

        robot.light.setPosition(1);

        while (opModeInInit()) {

            robot.hood.setPosition(shoot0Hood);

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;
            }

            if (redAlliance) {
                robot.light.setPosition(0.28);

                // ---- FIRST SHOT ----
                x1 = rx1;
                y1 = ry1;
                h1 = rh1;

                // ---- PICKUP PATH ----
                x2a = rx2a;
                y2a = ry2a;
                h2a = rh2a;
                x2b = rx2b;
                y2b = ry2b;
                h2b = rh2b;
                x3a = rx3a;
                y3a = ry3a;
                h3a = rh3a;
                x3b = rx3b;
                y3b = ry3b;
                h3b = rh3b;
                x4a = rx4a;
                y4a = ry4a;
                h4a = rh4a;
                x4b = rx4b;
                y4b = ry4b;
                h4b = rh4b;
                xPrep = rxPrep;
                yPrep = ryPrep;
                hPrep = rhPrep;
                xShoot = rShootX;
                yShoot = rShootY;
                hShoot = rShootH;


            } else {
                robot.light.setPosition(0.6);

                // ---- FIRST SHOT ----
                x1 = bx1;
                y1 = by1;
                h1 = bh1;

                // ---- PICKUP PATH ----
                x2a = bx2a;
                y2a = by2a;
                h2a = bh2a;
                x2b = bx2b;
                y2b = by2b;
                h2b = bh2b;
                x3a = bx3a;
                y3a = by3a;
                h3a = bh3a;
                x3b = bx3b;
                y3b = by3b;
                h3b = bh3b;
                x4a = bx4a;
                y4a = by4a;
                h4a = bh4a;
                x4b = bx4b;
                y4b = by4b;
                h4b = bh4b;

                xPrep = bxPrep;
                yPrep = byPrep;
                hPrep = bhPrep;
                xShoot = bShootX;
                yShoot = bShootY;
                hShoot = bShootH;

            }

            shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .strafeToLinearHeading(new Vector2d(x1, y1), h1);

            pickup1 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                    .strafeToLinearHeading(new Vector2d(x2a, y2a), h2a)
                    .strafeToLinearHeading(new Vector2d(x2b, y2b), h2b,
                            new TranslationalVelConstraint(pickup1Speed));

            shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, h2b))
                    .strafeToLinearHeading(new Vector2d(xPrep, yPrep), hPrep)
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), hShoot);

            pickup2 = drive.actionBuilder(new Pose2d(xShoot, yShoot, hShoot))
                    .strafeToLinearHeading(new Vector2d(x3a, y3a), h3a)
                    .strafeToLinearHeading(new Vector2d(x3b, y3b), h3b,
                            new TranslationalVelConstraint(pickup1Speed));

            shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, h3b))
                    .strafeToLinearHeading(new Vector2d(xPrep, yPrep), hPrep)
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), hShoot);

            pickup3 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                    .strafeToLinearHeading(new Vector2d(x4a, y4a), h4a)
                    .strafeToLinearHeading(new Vector2d(x4b, y4b), h4b,
                            new TranslationalVelConstraint(pickup1Speed));
            shoot3 = drive.actionBuilder(new Pose2d(x4b, y4b, h4b))
                    .strafeToLinearHeading(new Vector2d(xPrep, yPrep), hPrep)
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), hShoot);



            TELE.addData("Red?", redAlliance);
            TELE.update();
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

            Actions.runBlocking(
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
                            intake(intake1Time)

                    )
            );

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
                            prepareShootAll(shoot1Time)
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
                            shootAllAuto(shootAllTime, spindexerSpeedIncrease)
                    )

            );

            Actions.runBlocking(
                    new ParallelAction(
                            pickup2.build(),
                            manageShooterAuto(
                                    normalIntakeTime,
                                    x2b,
                                    y2b,
                                    pickup1XTolerance,
                                    pickup1YTolerance
                            ),
                            intake(normalIntakeTime)

                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            manageFlywheelAuto(
                                    shootCycleTime,
                                    0.501,
                                    0.501,
                                    0.501,
                                    0.501
                            ),
                            shoot2.build(),
                            prepareShootAll(shootCycleTime)
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
                            shootAllAuto(shootAllTime, spindexerSpeedIncrease)
                    )

            );

            Actions.runBlocking(
                    new ParallelAction(
                            pickup3.build(),
                            manageShooterAuto(
                                    normalIntakeTime,
                                    x2b,
                                    y2b,
                                    pickup1XTolerance,
                                    pickup1YTolerance
                            ),
                            intake(normalIntakeTime)

                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            manageFlywheelAuto(
                                    shootCycleTime,
                                    0.501,
                                    0.501,
                                    0.501,
                                    0.501
                            ),
                            shoot3.build(),
                            prepareShootAll(shootCycleTime)
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
                            shootAllAuto(shootAllTime, spindexerSpeedIncrease)
                    )

            );


            TELE.addLine("finished");
            TELE.update();

            sleep(2000);

        }

    }
}