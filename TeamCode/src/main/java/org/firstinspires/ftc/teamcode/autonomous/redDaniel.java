package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.*;
import static org.firstinspires.ftc.teamcode.utils.PositionalServoProgrammer.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.utils.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@Autonomous
public class redDaniel extends LinearOpMode {

    Robot robot;

    MultipleTelemetry TELE;

    MecanumDrive drive;

    AprilTagWebcam aprilTag;

    Flywheel flywheel;

    double velo = 0.0;
    double targetVelocity = 0.0;
    public static double intake1Time = 6.5;

    public static double intake2Time = 6.5;

    public static double colorDetect = 3.0;

    boolean gpp = false;

    boolean pgp = false;

    boolean ppg = false;

    double powPID = 0.0;

    int b1 = 0; // 0 = no ball, 1 = green, 2 = purple

    int b2 = 0;// 0 = no ball, 1 = green, 2 = purple

    int b3 = 0;// 0 = no ball, 1 = green, 2 = purple

    boolean spindexPosEqual(double spindexer) {
        TELE.addData("Velocity", velo);
        TELE.addLine("spindex equal");
        TELE.update();
        return (scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) > spindexer - 0.01 &&
                scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) < spindexer + 0.01);
    }

    public Action initShooter(int vel) {
        return new Action() {
            double initPos = 0.0;
            double stamp = 0.0;
            double stamp1 = 0.0;
            double ticker = 0.0;
            double stamp2 = 0.0;
            double currentPos = 0.0;
            boolean steady = false;
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp2 = getRuntime();
                }

                targetVelocity = (double) vel;
                ticker++;
                if (ticker % 16 == 0) {
                    stamp = getRuntime();
                    stamp1 = stamp;
                }

                powPID = flywheel.manageFlywheel(AUTO_CLOSE_VEL, (double) robot.shooter1.getCurrentPosition());
                velo = flywheel.getVelo();
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);
                robot.transfer.setPower(1);

                TELE.addData("Velocity", velo);
                TELE.update();
                 if (vel < velo && getRuntime() - stamp2 > 3.0 && !steady){
                     steady = true;
                     stamp2 = getRuntime();
                     return true;
                 } else if (steady && getRuntime() - stamp2 > 1.5){
                     TELE.addData("Velocity", velo);
                     TELE.addLine("finished init");
                     TELE.update();
                     return false;
                 } else {
                     return true;
                 }
            }
        };
    }

    public Action steadyShooter(int vel, boolean last) {
        return new Action() {
            double stamp = 0.0;
            boolean steady = false;
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                powPID = flywheel.manageFlywheel(AUTO_CLOSE_VEL, (double) robot.shooter1.getCurrentPosition());
                velo = flywheel.getVelo();
                steady = flywheel.getSteady();
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);
                robot.transfer.setPower(1);

                TELE.addData("Velocity", velo);
                TELE.update();

                if (last && !steady){
                    stamp = getRuntime();
                    return false;
                } else if (steady) {
                    stamp = getRuntime();
                    return true;
                } else {
                    return true;
                }
            }
        };
    }

    public Action Obelisk() {
        return new Action() {
            double stamp = getRuntime();
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = getRuntime();
                }
                ticker++;

                if (aprilTag.getTagById(21) != null) {
                    gpp = true;
                } else if (aprilTag.getTagById(22) != null) {
                    pgp = true;
                } else if (aprilTag.getTagById(23) != null) {
                    ppg = true;
                }
                aprilTag.update();

                TELE.addData("Velocity", velo);
                TELE.addData("21", gpp);
                TELE.addData("22", pgp);
                TELE.addData("23", ppg);
                TELE.update();

                if (gpp || pgp || ppg){
                    robot.turr1.setPosition(turret_red);
                    robot.turr2.setPosition(1 - turret_red);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }

    public Action spindex (double spindexer, double vel){
        return new Action() {
            double currentPos = 0.0;
            double stamp = 0.0;
            double initPos = 0.0;
            double stamp1 = 0.0;
            int ticker = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ticker++;
                if (ticker % 8 == 0) {
                    currentPos = (double) robot.shooter1.getCurrentPosition() / 2048;
                    stamp = getRuntime();
                    velo = -60 * ((currentPos - initPos) / (stamp - stamp1));
                    initPos = currentPos;
                    stamp1 = stamp;
                }

                if (vel - velo > 500 && ticker > 16) {
                    powPID = 1.0;
                } else if (velo - vel > 500 && ticker > 16){
                    powPID = 0.0;
                } else if (Math.abs(vel - velo) < 100 && ticker > 16){
                    double feed = Math.log((668.39 / (vel + 591.96)) - 0.116) / -4.18;

                    // --- PROPORTIONAL CORRECTION ---
                    double error = vel - velo;
                    double correction = kP * error;

                    // limit how fast power changes (prevents oscillation)
                    correction = Math.max(-maxStep, Math.min(maxStep, correction));

                    // --- FINAL MOTOR POWER ---
                    powPID = feed + correction;

                    // clamp to allowed range
                    powPID = Math.max(0, Math.min(1, powPID));
                }

                powPID = flywheel.manageFlywheel(AUTO_CLOSE_VEL, (double) robot.shooter1.getCurrentPosition());
                velo = flywheel.getVelo();
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);
                robot.spin1.setPosition(spindexer);
                robot.spin2.setPosition(1-spindexer);
                TELE.addData("Velocity", velo);
                TELE.addLine("spindex");
                TELE.update();
                return !spindexPosEqual(spindexer);
            }
        };
    }

    public Action Shoot(double vel) {
        return new Action() {
            double transferStamp = 0.0;
            int ticker = 1;
            boolean transferIn = false;
            double currentPos = 0.0;
            double stamp = 0.0;
            double initPos = 0.0;
            double stamp1 = 0.0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                TELE.addData("Velocity", velo);
                TELE.addLine("shooting");
                TELE.update();

                if (ticker % 8 == 0) {
                    currentPos = (double) robot.shooter1.getCurrentPosition() / 2048;
                    stamp = getRuntime();
                    velo = -60 * ((currentPos - initPos) / (stamp - stamp1));
                    initPos = currentPos;
                    stamp1 = stamp;
                }

                if (vel - velo > 500 && ticker > 16) {
                    powPID = 1.0;
                } else if (velo - vel > 500 && ticker > 16){
                    powPID = 0.0;
                } else if (Math.abs(vel - velo) < 100 && ticker > 16){
                    double feed = Math.log((668.39 / (vel + 591.96)) - 0.116) / -4.18;

                    // --- PROPORTIONAL CORRECTION ---
                    double error = vel - velo;
                    double correction = kP * error;

                    // limit how fast power changes (prevents oscillation)
                    correction = Math.max(-maxStep, Math.min(maxStep, correction));

                    // --- FINAL MOTOR POWER ---
                    powPID = feed + correction;

                    // clamp to allowed range
                    powPID = Math.max(0, Math.min(1, powPID));
                }

                powPID = flywheel.manageFlywheel(AUTO_CLOSE_VEL, (double) robot.shooter1.getCurrentPosition());
                velo = flywheel.getVelo();
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);


                if (ticker == 1) {
                    transferStamp = getRuntime();
                    ticker++;
                }
                if (getRuntime() - transferStamp > waitTransfer && !transferIn) {
                    robot.transferServo.setPosition(transferServo_in);
                    TELE.addData("Velocity", velo);
                    TELE.addData("ticker", ticker);
                    TELE.update();
                    transferIn = true;
                    return true;
                } else if (getRuntime() - transferStamp > waitTransfer+waitTransferOut && transferIn){
                    robot.transferServo.setPosition(transferServo_out);
                    TELE.addData("Velocity", velo);
                    TELE.addLine("shot once");
                    TELE.update();
                    return false;
                } else {
                    return true;
                }

            }
        };
    }

    public Action intake(double intakeTime) {
        return new Action() {
            double position = 0.0;
            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = getRuntime();
                }
                ticker++;

                double s1D = robot.color1.getDistance(DistanceUnit.MM);
                double s2D = robot.color2.getDistance(DistanceUnit.MM);
                double s3D = robot.color3.getDistance(DistanceUnit.MM);

                if ((getRuntime() % 0.3) > 0.15) {
                    position = spindexer_intakePos1 + 0.02;
                } else {
                    position = spindexer_intakePos1 - 0.02;
                }
                robot.spin1.setPosition(position);
                robot.spin2.setPosition(1 - position);

                TELE.addData("Velocity", velo);
                TELE.addLine("Intaking");
                TELE.update();

                robot.intake.setPower(1);
                if ((s1D < 40.0 && s2D < 40.0 && s3D < 40.0) || getRuntime() - stamp > intakeTime) {
                    robot.intake.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }

    public Action ColorDetect() {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;
            double position = 0.0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = getRuntime();
                }
                ticker++;

                if ((getRuntime() % 0.3) > 0.15) {
                    position = spindexer_intakePos1 + 0.02;
                } else {
                    position = spindexer_intakePos1 - 0.02;
                }
                robot.spin1.setPosition(position);
                robot.spin2.setPosition(1 - position);

                double s1D = robot.color1.getDistance(DistanceUnit.MM);
                double s2D = robot.color2.getDistance(DistanceUnit.MM);
                double s3D = robot.color3.getDistance(DistanceUnit.MM);

                if (s1D < 40) {

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

                if (s2D < 40) {

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

                if (s3D < 30) {

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

        flywheel = new Flywheel();

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        drive = new MecanumDrive(hardwareMap, new Pose2d(
                0, 0, 0
        ));

        aprilTag = new AprilTagWebcam();

        TrajectoryActionBuilder shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

        TrajectoryActionBuilder pickup1 = drive.actionBuilder(new Pose2d(rx1, ry1, rh1))
                .strafeToLinearHeading(new Vector2d(rx2a, ry2a), rh2a)
                .strafeToLinearHeading(new Vector2d(rx2b, ry2b), rh2b);

        TrajectoryActionBuilder shoot1 = drive.actionBuilder(new Pose2d(rx2b, ry2b, rh2b))
                .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

        TrajectoryActionBuilder pickup2 = drive.actionBuilder(new Pose2d(rx1, ry1, rh1))

                .strafeToLinearHeading(new Vector2d(rx3a, ry3a), rh3a)

                .strafeToLinearHeading(new Vector2d(rx3b, ry3b), rh3b);

        TrajectoryActionBuilder shoot2 = drive.actionBuilder(new Pose2d(rx3b, ry3b, rh3b))
                .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

        aprilTag.init(robot, TELE);

        while (opModeInInit()) {

            if (gamepad2.dpadUpWasPressed()) {
                hoodDefault -= 0.01;
            }

            if (gamepad2.dpadDownWasPressed()) {
                hoodDefault += 0.01;
            }

            robot.hood.setPosition(hoodDefault);

            robot.turr1.setPosition(turret_detectRed);
            robot.turr2.setPosition(1 - turret_detectRed);

            robot.transferServo.setPosition(transferServo_out);

            robot.spin1.setPosition(spindexer_intakePos1);
            robot.spin2.setPosition(1 - spindexer_intakePos1);

            aprilTag.update();
            TELE.addData("Velocity", velo);
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            robot.hood.setPosition(hoodStart);

            Actions.runBlocking(
                    new ParallelAction(
                            shoot0.build(),
                            initShooter(AUTO_CLOSE_VEL),
                            Obelisk()
                    )
            );

            powPID = flywheel.manageFlywheel(AUTO_CLOSE_VEL, (double) robot.shooter1.getCurrentPosition());
            velo = flywheel.getVelo();
            robot.shooter1.setPower(powPID);
            robot.shooter2.setPower(powPID);

            shootingSequence();

            robot.hood.setPosition(hoodDefault);

            Actions.runBlocking(
                    new ParallelAction(
                            pickup1.build(),
                            intake(intake1Time)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            shoot1.build(),
                            ColorDetect(),
                            steadyShooter(AUTO_CLOSE_VEL, true)
                    )
            );

            powPID = flywheel.manageFlywheel(AUTO_CLOSE_VEL, (double) robot.shooter1.getCurrentPosition());
            velo = flywheel.getVelo();
            robot.shooter1.setPower(powPID);
            robot.shooter2.setPower(powPID);

            shootingSequence();

            Actions.runBlocking(
                    new ParallelAction(
                            pickup2.build(),
                            intake(intake2Time)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            shoot2.build(),
                            ColorDetect(),
                            steadyShooter(AUTO_CLOSE_VEL, true)
                    )
            );

            powPID = flywheel.manageFlywheel(AUTO_CLOSE_VEL, (double) robot.shooter1.getCurrentPosition());
            velo = flywheel.getVelo();
            robot.shooter1.setPower(powPID);
            robot.shooter2.setPower(powPID);

            shootingSequence();

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            TELE.addData("Velocity", velo);
            TELE.addLine("finished");
            TELE.update();

            sleep(2000);

        }

    }

    public void shootingSequence() {
        TELE.addData("Velocity", velo);
        if (gpp) {
            if (b1 + b2 + b3 == 4) {
                if (b1 == 2 && b2 - b3 == 0) {
                    sequence1();
                    TELE.addLine("sequence1");
                } else if (b2 == 2 && b1 - b3 == 0) {
                    sequence3();
                    TELE.addLine("sequence3");
                } else if (b3 == 2 && b1 - b2 == 0) {
                    sequence6();
                    TELE.addLine("sequence6");
                } else {
                    sequence1();
                    TELE.addLine("sequence1");
                }
            } else if (b1 + b2 + b3 >= 5) {
                if (b1 == 2) {
                    sequence1();
                    TELE.addLine("sequence1");
                } else if (b2 == 2) {
                    sequence3();
                    TELE.addLine("sequence3");
                } else if (b3 == 2) {
                    sequence6();
                    TELE.addLine("sequence6");
                }
            } else {
                sequence1();
                TELE.addLine("sequence1");
            }
        } else if (pgp) {
            if (b1 + b2 + b3 == 4) {
                if (b1 == 2 && b2 - b3 == 0) {
                    sequence3();
                    TELE.addLine("sequence3");
                } else if (b2 == 2 && b1 - b3 == 0) {
                    sequence1();
                    TELE.addLine("sequence1");
                } else if (b3 == 2 && b1 - b2 == 0) {
                    sequence4();
                    TELE.addLine("sequence4");
                } else {
                    sequence1();
                    TELE.addLine("sequence1");
                }
            } else if (b1 + b2 + b3 >= 5) {
                if (b1 == 2) {
                    sequence3();
                    TELE.addLine("sequence3");
                } else if (b2 == 2) {
                    sequence1();
                    TELE.addLine("sequence1");
                } else if (b3 == 2) {
                    sequence4();
                    TELE.addLine("sequence4");
                }
            } else {
                sequence3();
                TELE.addLine("sequence3");
            }
        } else if (ppg) {
            if (b1 + b2 + b3 == 4) {
                if (b1 == 2 && b2 - b3 == 0) {
                    sequence6();
                    TELE.addLine("sequence6");
                } else if (b2 == 2 && b1 - b3 == 0) {
                    sequence5();
                    TELE.addLine("sequence5");
                } else if (b3 == 2 && b1 - b2 == 0) {
                    sequence1();
                    TELE.addLine("sequence1");
                } else {
                    sequence1();
                    TELE.addLine("sequence1");
                }
            } else if (b1 + b2 + b3 >= 5) {
                if (b1 == 2) {
                    sequence6();
                    TELE.addLine("sequence6");
                } else if (b2 == 2) {
                    sequence5();
                    TELE.addLine("sequence5");
                } else if (b3 == 2) {
                    sequence1();
                    TELE.addLine("sequence1");
                }
            } else {
                sequence6();
                TELE.addLine("sequence6");
            }
        } else {
            sequence1();
            TELE.addLine("sequence1");
        }
        TELE.update();
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
