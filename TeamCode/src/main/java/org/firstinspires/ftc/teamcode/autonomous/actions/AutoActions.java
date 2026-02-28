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

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.Color;
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
public class AutoActions {
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
    public static double firstSpindexShootPos = spinStartPos;
    private boolean shootForward = true;
    public int motif = 0;
    double spinEndPos = 0.95;
    private boolean intaking = false;
    public AutoActions(Robot rob, MecanumDrive dri, MultipleTelemetry tel, Servos ser, Flywheel fly, Spindexer spi, Targeting tar, Targeting.Settings tS, Turret tur, Light lig) {
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

            void spin1PosFirst() {
                firstSpindexShootPos = spindexer_outtakeBall1;
                shootForward = true;
                spinEndPos = 0.95;
            }

            void spin2PosFirst() {
                firstSpindexShootPos = spindexer_outtakeBall2;
                shootForward = false;
                spinEndPos = 0.05;
            }

            void reverseSpin2PosFirst() {
                firstSpindexShootPos = spindexer_outtakeBall2;
                shootForward = true;
                spinEndPos = 0.95;
            }

            void spin3PosFirst() {
                firstSpindexShootPos = spindexer_outtakeBall3;
                shootForward = false;
                spinEndPos = 0.05;
            }

            void oddSpin3PosFirst() {
                firstSpindexShootPos = spindexer_outtakeBall3b;
                shootForward = true;
                spinEndPos = 0.95;
            }

            Action manageShooter = null;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                    manageShooter = manageShooterAuto(time, posX, posY, posH, false);
                    driverSlotGreen = 0;
                    passengerSlotGreen = 0;
                    rearSlotGreen = 0;
                }
                ticker++;
                servos.setTransferPos(transferServo_out);
                drive.updatePoseEstimate();

                light.setState(StateEnums.LightState.GOAL_LOCK);

                teleStart = drive.localizer.getPose();

                manageShooter.run(telemetryPacket);

                TELE.addData("Most Green Slot", mostGreenSlot);
                TELE.addData("Driver Slot Greeness", driverSlotGreen);
                TELE.addData("Passenger Slot Greeness", passengerSlotGreen);
                TELE.addData("Rear Greeness", rearSlotGreen);
                TELE.update();

                if ((System.currentTimeMillis() - stamp) < (colorSenseTime * 1000)) {

                    spindexerWiggle *= -1.0;

                    servos.setSpinPos(spindexer_intakePos1 + spindexerWiggle);


// Rear Center (Position 1)
                    double distanceRearCenter = robot.color1.getDistance(DistanceUnit.MM);
                    if (distanceRearCenter < 52) {
                        NormalizedRGBA color1RGBA = robot.color1.getNormalizedColors();
                        double gP1 = color1RGBA.green / (color1RGBA.green + color1RGBA.red + color1RGBA.blue);
                        if (gP1 >= 0.38) {
                            rearSlotGreen++;
                        }
                    }

// Front Driver (Position 2)
                    double distanceFrontDriver = robot.color2.getDistance(DistanceUnit.MM);
                    if (distanceFrontDriver < 50) {
                        NormalizedRGBA color2RGBA = robot.color2.getNormalizedColors();
                        double gP2 = color2RGBA.green / (color2RGBA.green + color2RGBA.red + color2RGBA.blue);
                        if (gP2 >= 0.4) {
                            driverSlotGreen++;
                        }
                    }

// Front Passenger (Position 3)
                    double distanceFrontPassenger = robot.color3.getDistance(DistanceUnit.MM);
                    if (distanceFrontPassenger < 29) {
                        NormalizedRGBA color3RGBA = robot.color3.getNormalizedColors();
                        double gP3 = color3RGBA.green / (color3RGBA.green + color3RGBA.red + color3RGBA.blue);
                        if (gP3 >= 0.4) {
                            passengerSlotGreen++;
                        }
                    }

                    spindexer.setIntakePower(-0.1);

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
                            spinEndPos = 0.95;
                        } else if (mostGreenSlot == 2) {
                            firstSpindexShootPos = spindexer_outtakeBall2;
                            shootForward = false;
                            spinEndPos = 0.05;
                        } else {
                            firstSpindexShootPos = spindexer_outtakeBall3b;
                            shootForward = true;
                            spinEndPos = 0.95;
                        }
                    } else if (motif_id == 22) {
                        if (mostGreenSlot == 1) {
                            firstSpindexShootPos = spindexer_outtakeBall3b;
                            shootForward = true;
                            spinEndPos = 0.95;
                        } else if (mostGreenSlot == 2) {
                            firstSpindexShootPos = spindexer_outtakeBall1;
                            shootForward = true;
                            spinEndPos = 0.95;
                        } else {
                            firstSpindexShootPos = spindexer_outtakeBall1;
                            shootForward = false;
                            spinEndPos = 0.03;
                        }

                    } else {
                        if (mostGreenSlot == 1) {
                            firstSpindexShootPos = spindexer_outtakeBall3;
                            shootForward = false;
                            spinEndPos = 0.05;
                        } else if (mostGreenSlot == 2) {
                            firstSpindexShootPos = spindexer_outtakeBall3b;
                            shootForward = true;
                            spinEndPos = 0.95;
                        } else {
                            firstSpindexShootPos = spindexer_outtakeBall1;
                            shootForward = true;
                            spinEndPos = 0.95;                        }
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

    public Action shootAllAuto(double shootTime, double spindexSpeed, double posX, double posY, double posH) {
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
                    manageShooter = manageShooterAuto(shootTime, posX, posY, posH, false);

                }
                ticker++;

                manageShooter.run(telemetryPacket);

                double prevSpinPos = servos.getSpinCmdPos();

                boolean end;
                if (shootForward) {
                    end = servos.getSpinPos() > spinEndPos;
                } else {
                    end = servos.getSpinPos() < spinEndPos;
                }

                if (System.currentTimeMillis() - stamp < shootTime * 1000 && (!end || shooterTicker < Spindexer.waitFirstBallTicks + 1)) {

                    if (!servos.spinEqual(firstSpindexShootPos) && shooterTicker < 1) {
                        servos.setSpinPos(firstSpindexShootPos);
                    } else {
                        servos.setTransferPos(transferServo_in);
                        shooterTicker++;
                        Spindexer.whileShooting = true;
                        if (shootForward && shooterTicker > Spindexer.waitFirstBallTicks) {
                            servos.setSpinPos(prevSpinPos + spindexSpeed);
                        } else if (shooterTicker > Spindexer.waitFirstBallTicks) {
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

    public Action shootAllManual(
            double shootTime,
            double hoodMoveTime, //Set to 0.501 to show that you are not using, but you must set hoodPoses equal
            double spindexSpeed,
            double velStart,
            double hoodStart,
            double velEnd,
            double hoodEnd,
            double turr) {
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
                    manageShooter = manageShooterManual(shootTime, hoodMoveTime, velStart, hoodStart, velEnd, hoodEnd, turr);

                }
                ticker++;

                manageShooter.run(telemetryPacket);

                double prevSpinPos = servos.getSpinCmdPos();

                boolean end;
                if (shootForward) {
                    end = prevSpinPos > spinEndPos;
                } else {
                    end = prevSpinPos < spinEndPos;
                }

                if (System.currentTimeMillis() - stamp < shootTime * 1000 && !end) {
                    servos.setTransferPos(transferServo_in);
                    shooterTicker++;
                    Spindexer.whileShooting = true;
                    if (shootForward && shooterTicker > Spindexer.waitFirstBallTicks) {
                        servos.setSpinPos(prevSpinPos + spindexSpeed);
                    } else if (shooterTicker > Spindexer.waitFirstBallTicks) {
                        servos.setSpinPos(prevSpinPos - spindexSpeed);
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
                    manageShooter = manageShooterAuto(time, posX, posY, posH, false);
                }
                ticker++;

                spindexer.processIntake();
                spindexer.setIntakePower(1);
                light.setState(StateEnums.LightState.BALL_COUNT);
                light.update();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                manageShooter.run(telemetryPacket);

                if ((System.currentTimeMillis() - stamp) > (time * 1000)) {
                    servos.setSpinPos(spindexer_intakePos1);
                    intaking = false;
                    return false;
                } else {
                    intaking = true;
                    return true;
                }
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
            int prevMotif = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                detectingObelisk = true;
                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                    turret.pipelineSwitch(1);
                    ticker++;
                }

                motif = turret.detectObelisk();

                if (prevMotif == motif){
                    ticker++;
                }
                prevMotif = motif;

                turret.setTurret(turrPos);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || (xDone && yDone) || spindexer.isFull() || ticker > 10;

                teleStart = currentPose;

                if (shouldFinish) {
                    if (redAlliance) {
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
            double posH,
            boolean flywheelSensor
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

                    if (redAlliance) {
                        turret.pipelineSwitch(4);
                        light.setManualLightColor(Color.LightRed);
                    } else {
                        turret.pipelineSwitch(2);
                        light.setManualLightColor(Color.LightBlue);

                    }
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
                    deltaPose = new Pose2d(dx, dy, robotHeading);
                }
                Turret.limelightUsed = true;

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
                boolean shouldFinish = timeDone || (flywheel.getSteady() && flywheelSensor);

                teleStart = currentPose;

                TELE.addData("Steady?", flywheel.getSteady());
                TELE.update();

                return !shouldFinish;
            }
        };
    }

    public Action Wait(double time) {
        return new Action() {
            boolean ticker = false;
            double stamp = 0.0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!ticker) {
                    stamp = System.currentTimeMillis();
                    ticker = true;
                }

                return (System.currentTimeMillis() - stamp < time * 1000);

            }
        };
    }

    public Action manageShooterManual(
            double maxTime,
            double hoodMoveTime, //Set to 0.501 to show that you are not using, but you must set hoodPoses equal
            double velStart,
            double hoodStart,
            double velEnd,
            double hoodEnd,
            double turr
    ) {
        return new Action() {

            double stamp = 0.0;
            int ticker = 0;

            final boolean timeFallback = (maxTime != 0.501);

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
                if (turr == 0.501) {
                    deltaPose = new Pose2d(dx, dy, robotHeading);
                    if (!detectingObelisk) {
                        turret.trackGoal(deltaPose);
                    }
                } else {
                    turret.setTurret(turr);
                }

                servos.setHoodPos(hoodStart + ((hoodEnd - hoodStart) * Math.min(((System.currentTimeMillis() - stamp) / (hoodMoveTime * 1000)), 1)));
                double vel = velStart + (velEnd - velStart) * Math.min(((System.currentTimeMillis() - stamp) / (hoodMoveTime * 1000)), 1);

                double voltage = robot.voltage.getVoltage();
                flywheel.setPIDF(Robot.shooterPIDF_P, Robot.shooterPIDF_I, Robot.shooterPIDF_D, Robot.shooterPIDF_F / voltage);
                flywheel.manageFlywheel(vel);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > maxTime * 1000;

                teleStart = currentPose;

                TELE.addData("Steady?", flywheel.getSteady());
                TELE.update();

                return !timeDone;
            }
        };
    }

    public Action ShakeDrivetrain(
            double time
    ){
        return new Action() {
            int ticker = 0;
            double stamp = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0){
                    stamp = System.currentTimeMillis();
                }
                ticker++;

                double currentStamp = System.currentTimeMillis();
                if (currentStamp - stamp < time*1000 && (intaking || ticker < 50)) {
                    if (ticker % 10000 < 5000) {
                        robot.frontLeft.setPower(0.5);
                        robot.backLeft.setPower(0.5);
                        robot.frontRight.setPower(0.5);
                        robot.backRight.setPower(0.5);
                    } else {
                        robot.frontLeft.setPower(-0.5);
                        robot.backLeft.setPower(-0.5);
                        robot.frontRight.setPower(-0.5);
                        robot.backRight.setPower(-0.5);
                    }
                    return true;
                } else {
                    robot.frontLeft.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backRight.setPower(0);
                    return false;
                }
            }
        };
    }
}


