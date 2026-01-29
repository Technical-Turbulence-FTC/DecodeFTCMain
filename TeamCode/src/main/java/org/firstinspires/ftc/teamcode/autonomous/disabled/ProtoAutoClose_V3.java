package org.firstinspires.ftc.teamcode.autonomous.disabled;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh1;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bh4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx1;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.bx4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.by1;
import static org.firstinspires.ftc.teamcode.constants.Poses.by2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.by2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.by3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.by3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.by4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.by4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh1;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rh4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx1;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.rx4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry1;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry2a;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry2b;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry3a;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry3b;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry4a;
import static org.firstinspires.ftc.teamcode.constants.Poses.ry4b;
import static org.firstinspires.ftc.teamcode.constants.Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.hoodAuto;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.turret_blueClose;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.turret_redClose;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.AUTO_CLOSE_VEL;
import static org.firstinspires.ftc.teamcode.teleop.TeleopV3.spinPow;

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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;

import java.util.List;
@Disabled
@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class ProtoAutoClose_V3 extends LinearOpMode {
    public static double intake1Time = 2.7;
    public static double intake2Time = 3.0;
    public static double colorDetect = 3.0;
    public static double holdTurrPow = 0.01; // power to hold turret in place
    public static double slowSpeed = 30.0;
    Robot robot;
    MultipleTelemetry TELE;
    MecanumDrive drive;
    Flywheel flywheel;
    Servos servo;
    double velo = 0.0;
    boolean gpp = false;
    boolean pgp = false;
    boolean ppg = false;
    public static double spinUnjamTime = 0.6;
    double powPID = 0.0;
    double bearing = 0.0;
    int b1 = 0; // 0 = no ball, 1 = green, 2 = purple
    int b2 = 0;// 0 = no ball, 1 = green, 2 = purple
    int b3 = 0;// 0 = no ball, 1 = green, 2 = purple

    public Action initShooter(int vel) {
        return new Action() {
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flywheel.manageFlywheel(vel);
                velo = flywheel.getVelo();

                TELE.addData("Velocity", velo);
                TELE.update();

                return !flywheel.getSteady();
            }
        };
    }

    public Action Obelisk() {
        return new Action() {
            int id = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                LLResult result = robot.limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        id = fiducial.getFiducialId();
                        TELE.addData("ID", id);
                        TELE.update();
                    }

                }

                if (id == 21) {
                    gpp = true;
                } else if (id == 22) {
                    pgp = true;
                } else if (id == 23) {
                    ppg = true;
                }

                TELE.addData("Velocity", velo);
                TELE.addData("21", gpp);
                TELE.addData("22", pgp);
                TELE.addData("23", ppg);
                TELE.update();

                if (gpp || pgp || ppg) {
                    if (redAlliance) {
                        robot.limelight.pipelineSwitch(3);
                        robot.turr1.setPosition(turret_redClose);
                        robot.turr2.setPosition(1 - turret_redClose);
                        return false;

                    } else {
                        robot.limelight.pipelineSwitch(2);
                        double turretPID = turret_blueClose;
                        robot.turr1.setPosition(turretPID);
                        robot.turr2.setPosition(1 - turretPID);
                        return false;
                    }
                } else {
                    return true;
                }
            }
        };
    }

    public Action spindex(double spindexer, int vel) {
        return new Action() {
            double spinPID = 0.0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flywheel.manageFlywheel(vel);
                velo = flywheel.getVelo();

                spinPID = servo.setSpinPos(spindexer);
                robot.spin1.setPosition(spinPID);
                robot.spin2.setPosition(-spinPID);
                TELE.addData("Velocity", velo);
                TELE.addLine("spindex");
                TELE.update();

                drive.updatePoseEstimate();
                teleStart = drive.localizer.getPose();

                if (servo.spinEqual(spindexer)) {
                    robot.spin1.setPosition(0);
                    robot.spin2.setPosition(0);

                    return false;
                } else {
                    return true;
                }
            }
        };
    }


    public Action Shoot(int vel) {
        return new Action() {
            int ticker = 1;
            double initPos = 0.0;
            double finalPos = 0.0;
            boolean zeroNeeded = false;
            boolean zeroPassed = false;
            double currentPos = 0.0;
            double prevPos = 0.0;
            double stamp = 0.0;

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

                if (getRuntime() - stamp < 2.7) {

                    robot.transferServo.setPosition(transferServo_in);

                    robot.spin1.setPosition(-spinPow);
                    robot.spin2.setPosition(spinPow);
                    return true;

                } else {
                    robot.transferServo.setPosition(transferServo_out);
                    return false;
                }

            }
        };
    }

    public Action spindexUnjam(double jamTime) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {



                ticker++;

                if (ticker == 1) {
                    stamp = getRuntime();
                }

                if (ticker % 12 < 6) {

                    robot.spin1.setPosition(-1);
                    robot.spin2.setPosition(1);

                } else {
                    robot.spin1.setPosition(1);
                    robot.spin2.setPosition(-1);
                }

                if (getRuntime() - stamp > jamTime+0.4) {

                    robot.intake.setPower(0.5);

                    return false;
                }
                else if (getRuntime() - stamp > jamTime) {

                    robot.intake.setPower(-(getRuntime()-stamp-jamTime)*2.5);

                    return true;
                }

                else {
                    robot.intake.setPower(1);
                    return true;
                }
            }
        };
    }

    public Action intake(double intakeTime) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;
            double spinCurrentPos = 0.0;
            double spinInitPos = 0.0;
            boolean reverse = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = getRuntime();
                }
                ticker++;

                if (ticker % 60 < 12) {

                    robot.spin1.setPosition(-1);
                    robot.spin2.setPosition(1);

                } else if (ticker % 60 < 30) {
                    robot.spin1.setPosition(-0.5);
                    robot.spin2.setPosition(0.5);
                }
                else if (ticker % 60 < 42) {
                    robot.spin1.setPosition(1);
                    robot.spin2.setPosition(-1);
                }
                else {
                    robot.spin1.setPosition(0.5);
                    robot.spin2.setPosition(-0.5);
                }
                robot.intake.setPower(1);
                TELE.addData("Reverse?", reverse);
                TELE.update();

                if (getRuntime() - stamp > intakeTime) {
                    if (reverse) {
                        robot.spin1.setPosition(-1);
                        robot.spin2.setPosition(1);
                    } else {
                        robot.spin1.setPosition(1);
                        robot.spin2.setPosition(-1);
                    }
                    return false;
                } else {
                    if (ticker % 4 == 0) {
                        spinCurrentPos = servo.getSpinPos();
                        reverse = Math.abs(spinCurrentPos - spinInitPos) < 0.03;
                        spinInitPos = spinCurrentPos;
                    }

                    return true;
                }
            }
        };
    }

    public Action ColorDetect(int vel) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = getRuntime();
                }
                ticker++;

                flywheel.manageFlywheel(vel);
                velo = flywheel.getVelo();

                double s1D = robot.color1.getDistance(DistanceUnit.MM);
                double s2D = robot.color2.getDistance(DistanceUnit.MM);
                double s3D = robot.color3.getDistance(DistanceUnit.MM);

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                if (s1D < 43) {

                    double green = robot.color1.getNormalizedColors().green;
                    double red = robot.color1.getNormalizedColors().red;
                    double blue = robot.color1.getNormalizedColors().blue;

                    double gP = green / (green + red + blue);

                    if (gP >= 0.4) {
                        b1 = 2;
                    } else {
                        b1 = 1;
                    }
                }

                if (s2D < 60) {

                    double green = robot.color2.getNormalizedColors().green;
                    double red = robot.color2.getNormalizedColors().red;
                    double blue = robot.color2.getNormalizedColors().blue;

                    double gP = green / (green + red + blue);

                    if (gP >= 0.4) {
                        b2 = 2;
                    } else {
                        b2 = 1;
                    }
                }

                if (s3D < 33) {

                    double green = robot.color3.getNormalizedColors().green;
                    double red = robot.color3.getNormalizedColors().red;
                    double blue = robot.color3.getNormalizedColors().blue;

                    double gP = green / (green + red + blue);

                    if (gP >= 0.4) {
                        b3 = 2;
                    } else {
                        b3 = 1;
                    }
                }

                TELE.addData("Velocity", velo);
                TELE.addLine("Detecting");
                TELE.addData("Distance 1", s1D);
                TELE.addData("Distance 2", s2D);
                TELE.addData("Distance 3", s3D);
                TELE.addData("B1", b1);
                TELE.addData("B2", b2);
                TELE.addData("B3", b3);
                TELE.update();

                return (b1 + b2 + b3 < 4) && !(getRuntime() - stamp > colorDetect);
            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        flywheel = new Flywheel(hardwareMap);

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        drive = new MecanumDrive(hardwareMap, new Pose2d(
                0, 0, 0
        ));

        servo = new Servos(hardwareMap);

        robot.limelight.pipelineSwitch(1);
        robot.limelight.start();

        TrajectoryActionBuilder shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

        TrajectoryActionBuilder pickup1 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                .strafeToLinearHeading(new Vector2d(bx2a, by2a), bh2a)
                .strafeToLinearHeading(new Vector2d(bx2b, by2b), bh2b,
                new TranslationalVelConstraint(slowSpeed));
//
//        TrajectoryActionBuilder lever = drive.actionBuilder(new Pose2d(bx2b, by2b, bh2b))
//                .strafeToLinearHeading(new Vector2d(bx2c, by2c), bh2c);

        TrajectoryActionBuilder shoot1 = drive.actionBuilder(new Pose2d(bx2b, by2b, bh2b))
                .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

        TrajectoryActionBuilder pickup2 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                .strafeToLinearHeading(new Vector2d(bx3a, by3a), bh3a)
                .strafeToLinearHeading(new Vector2d(bx3b, by3b), bh3b,
                        new TranslationalVelConstraint(slowSpeed));

        TrajectoryActionBuilder shoot2 = drive.actionBuilder(new Pose2d(bx3b, by3b, bh3b))
                .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

        TrajectoryActionBuilder pickup3 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                .strafeToLinearHeading(new Vector2d(bx4a, by4a), bh4a)
                .strafeToLinearHeading(new Vector2d(bx4b, by4b), bh4b,
                        new TranslationalVelConstraint(slowSpeed));
        TrajectoryActionBuilder shoot3 = drive.actionBuilder(new Pose2d(bx4b, by4b, bh4b))
                .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

        while (opModeInInit()) {

            if (gamepad2.dpadUpWasPressed()) {
                hoodAuto -= 0.01;
            }

            if (gamepad2.dpadDownWasPressed()) {
                hoodAuto += 0.01;
            }

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;

            }

            double turretPID;
            if (redAlliance) {
                turretPID = turret_redClose;

                shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

                pickup1 = drive.actionBuilder(new Pose2d(rx1, ry1, rh1))
                        .strafeToLinearHeading(new Vector2d(rx2a, ry2a), rh2a)
                        .strafeToLinearHeading(new Vector2d(rx2b, ry2b), rh2b,
                                new TranslationalVelConstraint(slowSpeed));

//                lever = drive.actionBuilder(new Pose2d(rx2b, ry2b, rh2b))
//                        .strafeToLinearHeading(new Vector2d(rx2c, ry2c), rh2c);

                shoot1 = drive.actionBuilder(new Pose2d(rx2b, ry2b, rh2b))
                        .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

                pickup2 = drive.actionBuilder(new Pose2d(rx1, ry1, rh1))
                        .strafeToLinearHeading(new Vector2d(rx3a, ry3a), rh3a)
                        .strafeToLinearHeading(new Vector2d(rx3b, ry3b), rh3b,
                                new TranslationalVelConstraint(slowSpeed));

                shoot2 = drive.actionBuilder(new Pose2d(rx3b, ry3b, rh3b))
                        .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

                pickup3 = drive.actionBuilder(new Pose2d(rx1, ry1, rh1))
                        .strafeToLinearHeading(new Vector2d(rx4a, ry4a), rh4a)
                        .strafeToLinearHeading(new Vector2d(rx4b, ry4b), rh4b,
                                new TranslationalVelConstraint(slowSpeed));
                shoot3 = drive.actionBuilder(new Pose2d(rx4b, ry4b, rh4b))
                        .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

            } else {
                turretPID = turret_blueClose;

                shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

                pickup1 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                        .strafeToLinearHeading(new Vector2d(bx2a, by2a), bh2a)
                        .strafeToLinearHeading(new Vector2d(bx2b, by2b), bh2b,
                                new TranslationalVelConstraint(slowSpeed));

                shoot1 = drive.actionBuilder(new Pose2d(bx2b, by2b, bh2b))
                        .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

                pickup2 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                        .strafeToLinearHeading(new Vector2d(bx3a, by3a), bh3a)
                        .strafeToLinearHeading(new Vector2d(bx3b, by3b), bh3b,
                                new TranslationalVelConstraint(slowSpeed));

                shoot2 = drive.actionBuilder(new Pose2d(bx3b, by3b, bh3b))
                        .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

                pickup3 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                        .strafeToLinearHeading(new Vector2d(bx4a, by4a), bh4a)
                        .strafeToLinearHeading(new Vector2d(bx4b, by4b), bh4b,
                                new TranslationalVelConstraint(slowSpeed));
                shoot3 = drive.actionBuilder(new Pose2d(bx4b, by4b, bh4b))
                        .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);
            }

            robot.turr1.setPosition(turretPID);
            robot.turr2.setPosition(1 - turretPID);

            robot.hood.setPosition(hoodAuto);

            robot.transferServo.setPosition(transferServo_out);

            TELE.addData("Red?", redAlliance);
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            Actions.runBlocking(
                    new ParallelAction(
                            shoot0.build(),
                            initShooter(AUTO_CLOSE_VEL)
                    )
            );
            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            robot.transfer.setPower(1);

            shootingSequence();

            robot.transfer.setPower(0);

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            Actions.runBlocking(
                    new ParallelAction(
                            pickup1.build(),
                            intake(intake1Time)
                    )
            );
            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            Actions.runBlocking(
                    new SequentialAction(
                            shoot1.build(),
                            spindexUnjam(spinUnjamTime)

                    )
            );

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            robot.transfer.setPower(1);

            shootingSequence();

            robot.transfer.setPower(0);

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            Actions.runBlocking(
                    new ParallelAction(
                            pickup2.build(),
                            intake(intake2Time)
                    )
            );
            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            Actions.runBlocking(
                    new ParallelAction(
                            shoot2.build(),
                            spindexUnjam(spinUnjamTime)
                    )
            );

            robot.transfer.setPower(1);

            shootingSequence();

            robot.transfer.setPower(0);

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            Actions.runBlocking(
                    new ParallelAction(
                            pickup3.build(),
                            intake(intake2Time)
                    )
            );
            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            Actions.runBlocking(
                    new ParallelAction(
                            shoot3.build(),
                            spindexUnjam(spinUnjamTime)

                    )
            );

            robot.transfer.setPower(1);

            shootingSequence();

            robot.transfer.setPower(0);

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            TELE.addData("Velocity", velo);
            TELE.addLine("finished");
            TELE.update();

            sleep(2000);

        }

    }

    //TODO: adjust this according to Teleop numbers
    public void detectTag() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                bearing = result.getTx();
            }
        }
        double turretPos = (bearing / 1300);
        robot.turr1.setPosition(turretPos);
        robot.turr2.setPosition(1 - turretPos);
    }

    public void shootingSequence() {
        TELE.addLine("Shooting");
        TELE.update();
        Actions.runBlocking(Shoot(AUTO_CLOSE_VEL));
    }

    public void sequence1() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall1, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall2, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall3, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL)
                )
        );
    }

    public void sequence2() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall1, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall3, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall2, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL)
                )
        );
    }

    public void sequence3() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall2, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall1, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall3, AUTO_CLOSE_VEL),

                        Shoot(AUTO_CLOSE_VEL)
                )
        );
    }

    public void sequence4() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall2, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall3, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall1, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL)
                )
        );
    }

    public void sequence5() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall3, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall1, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall2, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL)
                )
        );
    }

    public void sequence6() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall3, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall2, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL),
                        spindex(spindexer_outtakeBall1, AUTO_CLOSE_VEL),
                        Shoot(AUTO_CLOSE_VEL)
                )
        );
    }
}