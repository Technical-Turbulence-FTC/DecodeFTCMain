package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.hoodOffset;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinEndPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3b;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Targeting.turretInterpolate;
import static org.firstinspires.ftc.teamcode.utils.Turret.limelightUsed;
import static org.firstinspires.ftc.teamcode.utils.Turret.turrDefault;

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

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.util.Objects;

@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class Auto_LT_Close extends LinearOpMode {
    public static double shoot0Vel = 2300, shoot0Hood = 0.93 + hoodOffset;
    public static double autoSpinStartPos = 0.2;
    public static double shoot0SpinSpeedIncrease = 0.02;

    public static double spindexerSpeedIncrease = 0.03;
    public static double finalSpindexerSpeedIncrease = 0.03;

    // These values are ADDED to turrDefault
    public static double redObeliskTurrPos1 = 0.12;
    public static double redObeliskTurrPos2 = 0.13;
    public static double redObeliskTurrPos3 = 0.14;
    public static double blueObeliskTurrPos1 = -0.12;
    public static double blueObeliskTurrPos2 = -0.13;
    public static double blueObeliskTurrPos3 = -0.14;
    public static double redTurretShootPos = 0.1;
    public static double blueTurretShootPos = -0.14;

    double obeliskTurrPos1 = 0.0;
    double obeliskTurrPos2 = 0.0;
    double obeliskTurrPos3 = 0.0;
    public static double normalIntakeTime = 3.3;
    public static double shoot1Turr = 0.57;
    public static double shoot0XTolerance = 1.0;
    double turretShootPos = 0.0;

    public static double finalShootAllTime = 3.0;
    public static double shootAllTime = 1.8;
    public static double shoot0Time = 1.6;
    public static double intake1Time = 3.3;
    public static double intake2Time = 3.8;

    public static double intake3Time = 4.2;

    public static double flywheel0Time = 3.5;
    public static double pickup1Speed = 15;
    // ---- SECOND SHOT / PICKUP ----
    public static double shoot1Vel = 2300;
    public static double shootAllVelocity = 2500;
    public static double shootAllHood = 0.78 + hoodOffset;
    // ---- PICKUP POSITION TOLERANCES ----
    public static double pickup1XTolerance = 2.0;
    public static double pickup1YTolerance = 2.0;
    // ---- OBELISK DETECTION ----
    public static double obelisk1Time = 1.5;
    public static double obelisk1XTolerance = 2.0;
    public static double obelisk1YTolerance = 2.0;
    public static double shoot1ToleranceX = 2.0;
    public static double shoot1ToleranceY = 2.0;
    public static double shoot1Time = 2;
    public static double shoot2Time = 2;
    public static double shoot3Time = 2;
    public static double colorSenseTime = 1.2;

    public static double firstShootTime = 0.3;
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
    private double firstSpindexShootPos = autoSpinStartPos;
    private boolean shootForward = true;
    double x1, y1, h1;

    double x2a, y2a, h2a, t2a;

    double x2b, y2b, h2b, t2b;
    double x2c, y2c, h2c, t2c;

    double x3a, y3a, h3a;
    double x3b, y3b, h3b;
    double x4a, y4a, h4a;
    double x4b, y4b, h4b;

    double xShoot, yShoot, hShoot;
    double xGate, yGate, hGate;
    double xPrep, yPrep, hPrep;
    double xLeave, yLeave, hLeave;

    private double shoot1Tangent;

    private int driverSlotGreen = 0;
    private int passengerSlotGreen = 0;

    private int rearSlotGreen = 0;
    private int mostGreenSlot = 0;
    int ballCycles = 3;
    int prevMotif = 0;

    public Action prepareShootAll(double colorSenseTime, double time, int motif_id) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            double spindexerWiggle = 0.01;

            boolean decideGreenSlot = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;
                servos.setTransferPos(transferServo_out);
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                double robX = drive.localizer.getPose().position.x;
                double robY = drive.localizer.getPose().position.y;
                double robotHeading = drive.localizer.getPose().heading.toDouble();

                double goalX = -15;
                double goalY = 0;

                double dx = robX - goalX;  // delta x from robot to goal
                double dy = robY - goalY;  // delta y from robot to goal
                Pose2d deltaPose = new Pose2d(dx, dy, robotHeading);

                double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

                targetingSettings = targeting.calculateSettings
                        (robX, robY, robotHeading, 0.0, turretInterpolate);

                turret.trackGoal(deltaPose);

                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.addData("motif", motif_id);
                TELE.update();

                if ((System.currentTimeMillis() - stamp) < (colorSenseTime * 1000)) {

                    spindexerWiggle *= -1.0;

                    servos.setSpinPos(spindexer_intakePos1 + spindexerWiggle);

                    spindexer.detectBalls(true, true);

                    if (Objects.equals(spindexer.GetFrontDriverColor(), Spindexer.BallColor.GREEN)) {
                        driverSlotGreen++;
                    }

                    if (Objects.equals(spindexer.GetFrontPassengerColor(), Spindexer.BallColor.GREEN)) {
                        passengerSlotGreen++;
                    }

                    if (Objects.equals(spindexer.GetRearCenterColor(), Spindexer.BallColor.GREEN)) {
                        rearSlotGreen++;
                    }

                    spindexer.setIntakePower(1);

                    decideGreenSlot = true;

                    return true;
                } else if (decideGreenSlot) {

                    if (driverSlotGreen >= passengerSlotGreen && driverSlotGreen >= rearSlotGreen) {
                        mostGreenSlot = 3;
                    } else if (passengerSlotGreen >= driverSlotGreen && passengerSlotGreen >= rearSlotGreen) {
                        mostGreenSlot = 2;
                    } else {
                        mostGreenSlot = 1;
                    }

                    decideGreenSlot = false;

                    if (motif_id == 21) {
                        if (mostGreenSlot == 1) {
                            firstSpindexShootPos = spindexer_outtakeBall1;
                            shootForward = true;
                        } else if (mostGreenSlot == 2) {
                            firstSpindexShootPos = spindexer_outtakeBall2;
                            shootForward = false;
                        } else {
                            firstSpindexShootPos = spindexer_outtakeBall3;
                            shootForward = false;
                        }
                    } else if (motif_id == 22) {
                        if (mostGreenSlot == 1) {
                            firstSpindexShootPos = spindexer_outtakeBall2;
                            shootForward = false;
                        } else if (mostGreenSlot == 2) {
                            firstSpindexShootPos = spindexer_outtakeBall3;
                            shootForward = false;
                        } else {
                            firstSpindexShootPos = spindexer_outtakeBall2;
                            shootForward = true;
                        }

                    } else {
                        if (mostGreenSlot == 1) {
                            firstSpindexShootPos = spindexer_outtakeBall3;
                            shootForward = false;
                        } else if (mostGreenSlot == 2) {
                            firstSpindexShootPos = spindexer_outtakeBall3b;
                            shootForward = true;
                        } else {
                            firstSpindexShootPos = spindexer_outtakeBall1;
                            shootForward = true;
                        }

                    }

                    return true;
                } else if ((System.currentTimeMillis() - stamp) < (time * 1000)) {
//                    TELE.addData("MostGreenSlot", mostGreenSlot);
//                    TELE.update();
                    spindexer.setIntakePower(-((System.currentTimeMillis() - stamp - colorSenseTime)) / 1000);

                    servos.setSpinPos(firstSpindexShootPos);

                    return true;
                } else {
                    return false;
                }

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

                double voltage = robot.voltage.getVoltage();
                flywheel.setPIDF(robot.shooterPIDF_P, robot.shooterPIDF_I, robot.shooterPIDF_D, robot.shooterPIDF_F / voltage);
                flywheel.manageFlywheel(vel);
                velo = flywheel.getVelo();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                spindexer.setIntakePower(-0.3);

                if (ticker == 1) {
                    stamp = getRuntime();
                }
                ticker++;

                spindexer.setIntakePower(0);
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                double robX = drive.localizer.getPose().position.x;
                double robY = drive.localizer.getPose().position.y;
                double robotHeading = drive.localizer.getPose().heading.toDouble();

                double goalX = -15;
                double goalY = 0;

                double dx = robX - goalX;  // delta x from robot to goal
                double dy = robY - goalY;  // delta y from robot to goal
                Pose2d deltaPose = new Pose2d(dx, dy, robotHeading);

                double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

                targetingSettings = targeting.calculateSettings
                        (robX, robY, robotHeading, 0.0, turretInterpolate);

                turret.trackGoal(deltaPose);

                if ((getRuntime() - stamp < shootTime && servos.getSpinPos() < spinEndPos) || shooterTicker == 0) {

                    if (shooterTicker == 0 && !servos.spinEqual(autoSpinStartPos)) {
                        servos.setSpinPos(autoSpinStartPos);
                    } else {
                        servos.setTransferPos(transferServo_in);
                        shooterTicker++;
                        double prevSpinPos = servos.getSpinCmdPos();
                        servos.setSpinPos(prevSpinPos + spindexSpeed);
                    }

                    return true;

                } else {
                    servos.setTransferPos(transferServo_out);

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

                teleStart = drive.localizer.getPose();

                spindexer.setIntakePower(-0.3);

                if (ticker == 1) {
                    stamp = getRuntime();
                }
                ticker++;

                spindexer.setIntakePower(0);
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                double robX = drive.localizer.getPose().position.x;
                double robY = drive.localizer.getPose().position.y;
                double robotHeading = drive.localizer.getPose().heading.toDouble();

                double goalX = -15;
                double goalY = 0;

                double dx = robX - goalX;  // delta x from robot to goal
                double dy = robY - goalY;  // delta y from robot to goal
                Pose2d deltaPose = new Pose2d(dx, dy, robotHeading);

                double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

                targetingSettings = targeting.calculateSettings
                        (robX, robY, robotHeading, 0.0, turretInterpolate);

                turret.trackGoal(deltaPose);

                if (getRuntime() - stamp < shootTime) {

                    if (getRuntime() - stamp < firstShootTime) {
                        servos.setTransferPos(transferServo_out);
                        servos.setSpinPos(firstSpindexShootPos);
                    } else {
                        servos.setTransferPos(transferServo_in);
                        shooterTicker++;
                        double prevSpinPos = servos.getSpinCmdPos();

                        if (shootForward) {
                            servos.setSpinPos(prevSpinPos + spindexSpeed);
                        } else {
                            servos.setSpinPos(prevSpinPos - spindexSpeed);
                        }
                    }

                    return true;

                } else {
                    servos.setTransferPos(transferServo_out);

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
                spindexer.setIntakePower(1);

                spindexer.ballCounterLight();
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                TELE.addData("Full?", spindexer.isFull());
                TELE.update();

                return ((System.currentTimeMillis() - stamp) < (intakeTime * 1000)) && !spindexer.isFull();
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
                    robot.limelight.pipelineSwitch(1);
                }

                ticker++;
                motif = turret.detectObelisk();

                turret.setTurret(turrPos);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || xDone || yDone;
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

                if (shouldFinish){
                    if (redAlliance){
                        robot.limelight.pipelineSwitch(4);
                    } else {
                        robot.limelight.pipelineSwitch(2);
                    }
                    return false;
                } else {
                    return true;
                }

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

                double voltage = robot.voltage.getVoltage();
                flywheel.setPIDF(robot.shooterPIDF_P, robot.shooterPIDF_I, robot.shooterPIDF_D, robot.shooterPIDF_F / voltage);
                flywheel.manageFlywheel(vel);
                servos.setHoodPos(hoodPos);

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

                servos.setHoodPos(targetingSettings.hoodAngle);

                double voltage = robot.voltage.getVoltage();
                flywheel.setPIDF(robot.shooterPIDF_P, robot.shooterPIDF_I, robot.shooterPIDF_D, robot.shooterPIDF_F / voltage);
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

                servos.setHoodPos(targetingSettings.hoodAngle);

                double voltage = robot.voltage.getVoltage();
                flywheel.setPIDF(robot.shooterPIDF_P, robot.shooterPIDF_I, robot.shooterPIDF_D, robot.shooterPIDF_F / voltage);
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

    // initialize path variables here
    TrajectoryActionBuilder shoot0 = null;
    TrajectoryActionBuilder pickup1 = null;
    TrajectoryActionBuilder shoot1 = null;
    TrajectoryActionBuilder pickup2 = null;
    TrajectoryActionBuilder shoot2 = null;
    TrajectoryActionBuilder pickup3 = null;
    TrajectoryActionBuilder shoot3 = null;

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

        turret = new Turret(robot, TELE, robot.limelight);

        turret.setTurret(turrDefault);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        servos.setSpinPos(autoSpinStartPos);

        servos.setTransferPos(transferServo_out);

        limelightUsed = false;

        robot.light.setPosition(1);

        while (opModeInInit()) {

            servos.setHoodPos(shoot0Hood);
            turret.setTurret(turrDefault);

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;
            }

            if (gamepad2.dpadLeftWasPressed()) {
                turrDefault -= 0.01;
            }

            if (gamepad2.dpadRightWasPressed()) {
                turrDefault += 0.01;
            }

            if (gamepad2.rightBumperWasPressed()){
                ballCycles++;
            }
            if (gamepad2.leftBumperWasPressed()){
                ballCycles--;
            }

            if (gamepad2.squareWasPressed()){
                robot.limelight.start();
                robot.limelight.pipelineSwitch(1);
                gamepad2.rumble(500);
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
                xLeave = rLeaveX;
                yLeave = rLeaveY;
                hLeave = rLeaveH;

                obeliskTurrPos1 = turrDefault + redObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + redObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + redObeliskTurrPos3;
                turretShootPos = turrDefault + redTurretShootPos;

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
                xLeave = bLeaveX;
                yLeave = bLeaveY;
                hLeave = bLeaveH;

                obeliskTurrPos1 = turrDefault + blueObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + blueObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + blueObeliskTurrPos3;
                turretShootPos = turrDefault + blueTurretShootPos;

            }

            shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(h1));

            pickup1 = drive.actionBuilder(new Pose2d(x1, y1, Math.toRadians(h1)))
                    .strafeToLinearHeading(new Vector2d(x2a, y2a), Math.toRadians(h2a))
                    .strafeToLinearHeading(new Vector2d(x2b, y2b), Math.toRadians(h2b),
                            new TranslationalVelConstraint(pickup1Speed));

            if (ballCycles < 2){
                shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, Math.toRadians(h2b)))
                        .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));
            } else {
                shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, Math.toRadians(h2b)))
                        .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));
            }

            pickup2 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(x3a, y3a), Math.toRadians(h3a))
                    .strafeToLinearHeading(new Vector2d(x3b, y3b), Math.toRadians(h3b),
                            new TranslationalVelConstraint(pickup1Speed));

            if (ballCycles < 3){
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));
            } else {
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hLeave));
            }

            pickup3 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(x4a, y4a), Math.toRadians(h4a))
                    .strafeToLinearHeading(new Vector2d(x4b, y4b), Math.toRadians(h4b),
                            new TranslationalVelConstraint(pickup1Speed));

            shoot3 = drive.actionBuilder(new Pose2d(x4b, y4b, Math.toRadians(h4b)))
                    .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));

            TELE.addData("Red?", redAlliance);
            TELE.addData("Turret Default", turrDefault);
            TELE.addData("Ball Cycles", ballCycles);

            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            robot.transfer.setPower(1);

            startAuto();

            if (ballCycles > 0){
                cycleStackClose();
            }

            if (ballCycles > 1){
                cycleStackMiddle();
            }

            if (ballCycles > 2){
                cycleStackFar();
            }

            while (opModeIsActive()) {

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                flywheel.manageFlywheel(0);

                TELE.addLine("finished");
                TELE.update();
            }

        }

    }

    void startAuto() {
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
    }

    void cycleStackClose(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup1.build(),
                        manageFlywheel(
                                shootAllVelocity,
                                shootAllHood,
                                intake1Time,
                                0.501,
                                0.501,
                                pickup1XTolerance,
                                pickup1YTolerance
                        ),
                        intake(intake1Time),
                        detectObelisk(
                                intake1Time,
                                0.501,
                                0.501,
                                obelisk1XTolerance,
                                obelisk1YTolerance,
                                obeliskTurrPos1
                        )

                )
        );

        motif = turret.getObeliskID();

        if (motif == 0) motif = 22;
        prevMotif = motif;

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
                        prepareShootAll(colorSenseTime, shoot1Time, motif)
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
    }

    void cycleStackMiddle(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup2.build(),
                        manageShooterAuto(
                                intake2Time,
                                0.501,
                                0.501,
                                pickup1XTolerance,
                                pickup1YTolerance
                        ),
                        intake(intake2Time),
                        detectObelisk(
                                intake2Time,
                                0.501,
                                0.501,
                                obelisk1XTolerance,
                                obelisk1YTolerance,
                                obeliskTurrPos2
                        )

                )
        );

        motif = turret.getObeliskID();

        if (motif == 0) motif = prevMotif;
        prevMotif = motif;

        Actions.runBlocking(
                new ParallelAction(
                        manageFlywheelAuto(
                                shoot2Time,
                                0.501,
                                0.501,
                                0.501,
                                0.501
                        ),
                        shoot2.build(),
                        prepareShootAll(colorSenseTime, shoot2Time, motif)
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
    }

    void cycleStackFar(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup3.build(),
                        manageShooterAuto(
                                intake3Time,
                                0.501,
                                0.501,
                                pickup1XTolerance,
                                pickup1YTolerance
                        ),
                        intake(intake3Time),
                        detectObelisk(
                                intake3Time,
                                0.501,
                                0.501,
                                obelisk1XTolerance,
                                obelisk1YTolerance,
                                obeliskTurrPos3
                        )

                )
        );

        motif = turret.getObeliskID();

        if (motif == 0) motif = prevMotif;
        prevMotif = motif;

        Actions.runBlocking(
                new ParallelAction(
                        manageFlywheelAuto(
                                shoot3Time,
                                0.501,
                                0.501,
                                0.501,
                                0.501
                        ),
                        shoot3.build(),
                        prepareShootAll(colorSenseTime, shoot3Time, motif)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        manageShooterAuto(
                                finalShootAllTime,
                                0.501,
                                0.501,
                                0.501,
                                0.501
                        ),
                        shootAllAuto(finalShootAllTime, finalSpindexerSpeedIncrease)
                )

        );
    }
}