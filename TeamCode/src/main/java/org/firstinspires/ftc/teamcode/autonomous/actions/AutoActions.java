package org.firstinspires.ftc.teamcode.autonomous.actions;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3b;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.constants.ServoPositions;
import org.firstinspires.ftc.teamcode.constants.StateEnums;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Light;
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
    Light light;
    Turret turret;
    private int driverSlotGreen = 0;
    private int passengerSlotGreen = 0;
    private int rearSlotGreen = 0;
    private int mostGreenSlot = 0;
    private double firstSpindexShootPos = spindexer_outtakeBall1;
    private boolean shootForward = true;
    public int motif = 0;
    double spinEndPos = ServoPositions.spinEndPos;

    public AutoActions(Robot rob, MecanumDrive dri, MultipleTelemetry tel, Servos ser, Flywheel fly, Spindexer spi, Targeting tar, Targeting.Settings tS, Turret tur, Light lig){
        this.robot = rob;
        this.drive = dri;
        this.TELE = tel;
        this.servos = ser;
        this.flywheel = fly;
        this.spindexer = spi;
        this.targeting = tar;
        this.targetingSettings = tS;
        this.turret = tur;
        this.light = lig;
    }

    public Action prepareShootAll(
            double colorSenseTime,
            double time,
            int motif_id,
            double posX,
            double posY,
            double posH
    ) {
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

            Action manageShooter = null;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                    manageShooter = manageShooterAuto(time, posX, posY, posH);
                }
                ticker++;
                servos.setTransferPos(transferServo_out);
                drive.updatePoseEstimate();

                light.setState(StateEnums.LightState.GOAL_LOCK);

                teleStart = drive.localizer.getPose();

                manageShooter.run(telemetryPacket);

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

    public Action shootAllAuto(double shootTime, double spindexSpeed) {
        return new Action() {
            int ticker = 1;

            double stamp = 0.0;

            int shooterTicker = 0;
            Action manageShooter = null;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                spindexer.setIntakePower(-0.1);

                light.setState(StateEnums.LightState.BALL_COLOR);
                light.update();

                if (ticker == 1) {
                    stamp = System.currentTimeMillis();
                    manageShooter = manageShooterAuto(shootTime, 0.501, 0.501, 0.501);

                }
                ticker++;

                manageShooter.run(telemetryPacket);

                double prevSpinPos = servos.getSpinCmdPos();

                boolean end;
                if (shootForward){
                    end = prevSpinPos > spinEndPos;
                } else {
                    end = prevSpinPos < spinEndPos;
                }

                if (System.currentTimeMillis() - stamp < shootTime*1000 && (!end || shooterTicker < Spindexer.waitFirstBallTicks+1)) {

                    if (!servos.spinEqual(firstSpindexShootPos) && shooterTicker < 1) {
                        servos.setTransferPos(transferServo_out);
                        servos.setSpinPos(firstSpindexShootPos);
                    } else {
                        servos.setTransferPos(transferServo_in);
                        shooterTicker++;
                        Spindexer.whileShooting = true;
                        if (shootForward && shooterTicker > Spindexer.waitFirstBallTicks) {
                            servos.setSpinPos(prevSpinPos + spindexSpeed);
                        } else if (shooterTicker > Spindexer.waitFirstBallTicks){
                            servos.setSpinPos(prevSpinPos - spindexSpeed);
                        }

                    }

                    return true;

                } else {
                    servos.setTransferPos(transferServo_out);
                    Spindexer.whileShooting = false;
                    spindexer.resetSpindexer();
                    spindexer.processIntake();

                    return false;

                }
            }
        };
    }

    public Action intake(
            double time,
            double posX,
            double posY,
            double posH
    ) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;
            Action manageShooter = null;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                    manageShooter = manageShooterAuto(time, posX, posY, posH);
                }
                ticker++;

                spindexer.processIntake();
                spindexer.setIntakePower(1);
                light.setState(StateEnums.LightState.BALL_COUNT);
                light.update();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                manageShooter.run(telemetryPacket);

                return ((System.currentTimeMillis() - stamp) < (time * 1000)) && !spindexer.isFull();
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
                    turret.pipelineSwitch(1);
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
                        turret.pipelineSwitch(4);
                    } else {
                        turret.pipelineSwitch(2);
                    }
                    detectingObelisk = false;
                    return false;
                } else {
                    return true;
                }

            }
        };
    }

    public Action manageShooterAuto(
            double time,
            double posX,
            double posY,
            double posH
    ) {

        return new Action() {

            double stamp = 0.0;
            int ticker = 0;

            final boolean timeFallback = (time != 0.501);

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
                if (posX != 0.501) {
                    deltaPose = new Pose2d(posX, posY, Math.toRadians(posH));
                } else {
                    deltaPose = new Pose2d(robotX, robotY, robotHeading);
                }

//                double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

                targetingSettings = targeting.calculateSettings
                        (robotX, robotY, robotHeading, 0.0, false);

                if (!detectingObelisk) {
                    turret.trackGoal(deltaPose);
                }

                servos.setHoodPos(targetingSettings.hoodAngle);

                double voltage = robot.voltage.getVoltage();
                flywheel.setPIDF(Robot.shooterPIDF_P, Robot.shooterPIDF_I, Robot.shooterPIDF_D, Robot.shooterPIDF_F / voltage);
                flywheel.manageFlywheel(targetingSettings.flywheelRPM);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean shouldFinish = timeDone || flywheel.getSteady();

                teleStart = currentPose;

                TELE.addData("Steady?", flywheel.getSteady());
                TELE.update();

                return !shouldFinish;
            }
        };
    }
}


