package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3b;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Targeting.turretInterpolate;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.constants.ServoPositions;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.util.Objects;

@Config
public class AutoActions{
    Robot robot;
    MultipleTelemetry TELE;
    Servos servos;
    Flywheel flywheel;
    MecanumDrive drive;
    Spindexer spindexer;
    Targeting targeting;
    Targeting.Settings targetingSettings;
    Turret turret;
    private int driverSlotGreen = 0;
    private int passengerSlotGreen = 0;
    private int rearSlotGreen = 0;
    private int mostGreenSlot = 0;
    private double firstSpindexShootPos = spinStartPos;
    private boolean shootForward = true;
    public static double firstShootTime = 0.3;
    public int motif = 0;
    double spinEndPos = ServoPositions.spinEndPos;

    public AutoActions(Robot rob, MecanumDrive dri, MultipleTelemetry tel, Servos ser, Flywheel fly, Spindexer spi, Targeting tar, Targeting.Settings tS, Turret tur){
        this.robot = rob;
        this.drive = dri;
        this.TELE = tel;
        this.servos = ser;
        this.flywheel = fly;
        this.spindexer = spi;
        this.targeting = tar;
        this.targetingSettings = tS;
        this.turret = tur;
    }

    public Action prepareShootAll(double colorSenseTime, double time, int motif_id) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            double spindexerWiggle = 0.01;

            boolean decideGreenSlot = false;

            void spin1PosFirst(){
                firstSpindexShootPos = spindexer_outtakeBall1;
                shootForward = true;
                spinEndPos = spindexer_outtakeBall3 + 0.1;
            }

            void spin2PosFirst(){
                firstSpindexShootPos = spindexer_outtakeBall2;
                shootForward = false;
                spinEndPos = spindexer_outtakeBall3b - 0.1;
            }

            void reverseSpin2PosFirst(){
                firstSpindexShootPos = spindexer_outtakeBall2;
                shootForward = true;
                spinEndPos = 0.95;
            }

            void spin3PosFirst(){
                firstSpindexShootPos = spindexer_outtakeBall3;
                shootForward = false;
                spinEndPos = spindexer_outtakeBall1 - 0.1;
            }

            void oddSpin3PosFirst(){
                firstSpindexShootPos = spindexer_outtakeBall3b;
                shootForward = true;
                spinEndPos = spindexer_outtakeBall2 + 0.1;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;
                servos.setTransferPos(transferServo_out);
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

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
                            spin1PosFirst();
                        } else if (mostGreenSlot == 2) {
                            spin2PosFirst();
                        } else {
                            spin3PosFirst();
                        }
                    } else if (motif_id == 22) {
                        if (mostGreenSlot == 1) {
                            spin2PosFirst();
                        } else if (mostGreenSlot == 2) {
                            spin3PosFirst();
                        } else {
                            reverseSpin2PosFirst();
                        }

                    } else {
                        if (mostGreenSlot == 1) {
                            spin3PosFirst();
                        } else if (mostGreenSlot == 2) {
                            oddSpin3PosFirst();
                        } else {
                            spin1PosFirst();
                        }
                    }

                    return true;
                } else if ((System.currentTimeMillis() - stamp) < (time * 1000)) {
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

                double voltage = robot.voltage.getVoltage();
                flywheel.setPIDF(robot.shooterPIDF_P, robot.shooterPIDF_I, robot.shooterPIDF_D, robot.shooterPIDF_F / voltage);
                flywheel.manageFlywheel(vel);
                velo = flywheel.getVelo();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                spindexer.setIntakePower(-0.1);

                if (ticker == 1) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;

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

                if ((System.currentTimeMillis() - stamp < shootTime*1000 && servos.getSpinPos() < 0.85) || shooterTicker == 0) {

                    if (shooterTicker == 0 && !servos.spinEqual(spinStartPos)) {
                        servos.setSpinPos(spinStartPos);
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

    private boolean doneShooting = false;
    public Action shootAllAuto(double shootTime, double spindexSpeed) {
        return new Action() {
            int ticker = 1;

            double stamp = 0.0;

            double velo = 0.0;

            int shooterTicker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                velo = flywheel.getVelo();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                spindexer.setIntakePower(-0.1);

                if (ticker == 1) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;

                double prevSpinPos = servos.getSpinCmdPos();

                boolean end;
                if (shootForward){
                    end = prevSpinPos > spinEndPos;
                } else {
                    end = prevSpinPos < spinEndPos;
                }

                if (System.currentTimeMillis() - stamp < shootTime*1000 && (!end || shooterTicker < 2)) {

                    if (!servos.spinEqual(firstSpindexShootPos) && shooterTicker < 3) {
                        servos.setTransferPos(transferServo_out);
                        servos.setSpinPos(firstSpindexShootPos);
                    } else {
                        servos.setTransferPos(transferServo_in);
                        shooterTicker++;

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
                    doneShooting = true;

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

                return ((System.currentTimeMillis() - stamp) < (intakeTime * 1000)) && !spindexer.isFull();
            }
        };
    }

    private boolean detectingObelisk = false;
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
                detectingObelisk = true;
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

                boolean shouldFinish = timeDone || (xDone && yDone) || spindexer.isFull();

                teleStart = currentPose;

                if (shouldFinish){
                    if (redAlliance){
                        robot.limelight.pipelineSwitch(4);
                    } else {
                        robot.limelight.pipelineSwitch(2);
                    }
                    detectingObelisk = false;
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

                teleStart = currentPose;

                return !shouldFinish;

            }
        };
    }

    public Action manageShooterAuto(
            double time,
            double posX,
            double posY,
            double posXTolerance,
            double posYTolerance,
            double posH,
            boolean whileIntaking
    ) {

        boolean timeFallback = (time != 0.501);
        boolean posXFallback = (posX != 0.501);
        boolean posYFallback = (posY != 0.501);

        return new Action() {

            double stamp = 0.0;
            int ticker = 0;
            int shootingTicker = 0;
            double shootingStamp = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }

                ticker++;

                double robotX = currentPose.position.x;
                double robotY = currentPose.position.y;

                double robotHeading = currentPose.heading.toDouble();

                double goalX = -15;
                double goalY = 0;

                double dx = robotX - goalX;  // delta x from robot to goal
                double dy = robotY - goalY;  // delta y from robot to goal
                Pose2d deltaPose;
                if (posX != 0.501){
                    deltaPose = new Pose2d(posX, posY, Math.toRadians(posH));
                } else {
                    deltaPose = new Pose2d(robotX, robotY, robotHeading);
                }

                double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

                targetingSettings = targeting.calculateSettings
                        (robotX, robotY, robotHeading, 0.0, false);

                if (!detectingObelisk){
                    turret.trackGoal(deltaPose);
                }

                servos.setHoodPos(targetingSettings.hoodAngle);

                double voltage = robot.voltage.getVoltage();
                flywheel.setPIDF(robot.shooterPIDF_P, robot.shooterPIDF_I, robot.shooterPIDF_D, robot.shooterPIDF_F / voltage);
                flywheel.manageFlywheel(targetingSettings.flywheelRPM);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(robotX - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(robotY - posY) < posYTolerance;
                boolean shouldFinish;
                if (whileIntaking){
                    shouldFinish = timeDone || (xDone && yDone) || spindexer.isFull();
                } else {
                    shouldFinish = timeDone || (xDone && yDone);
                }

                teleStart = currentPose;

                if (doneShooting && shootingTicker == 0){
                    shootingTicker++;
                    shootingStamp = System.currentTimeMillis();
                }

                if (System.currentTimeMillis() - shootingStamp > 100 || shouldFinish){
                    doneShooting = false;
                    return false;
                } else {
                    return true;
                }

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

                teleStart = currentPose;

                return !shouldFinish;

            }
        };
    }
}


