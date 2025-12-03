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
import org.firstinspires.ftc.teamcode.utils.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@Autonomous
public class redDaniel extends LinearOpMode {

    Robot robot;

    MultipleTelemetry TELE;

    MecanumDrive drive;

    AprilTagWebcam aprilTag;

    public static double intake1Time = 7.0;

    public static double intake2Time = 8.0;

    boolean gpp = false;

    boolean pgp = false;

    boolean ppg = false;

    int b1 = 0; // 0 = no ball, 1 = green, 2 = purple

    int b2 = 0;// 0 = no ball, 1 = green, 2 = purple

    int b3 = 0;// 0 = no ball, 1 = green, 2 = purple

    // Reset the counters after 1 second of not reading a ball.
    final double ColorCounterResetDelay = 1.0;
    // Number of times the loop needs to run before deciding on a color.
    final int ColorCounterTotalMinCount = 20;
    // If the color sensor reads a color this percentage of time
    // out of the total, declare the color.
    // Usage: (Color Count)/(Total Count) > ColorCounterThreshold
    final double ColorCounterThreshold = 0.65;

    boolean spindexPosEqual(double spindexer) {
        return (scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) > spindexer - 0.01 &&
                scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) < spindexer + 0.01);
    }

    boolean transferPosEqual(double transfer) {
        return (scalar * ((robot.transferServoPos.getVoltage() - restPos) / 3.3) >
                transfer - 0.01 &&
                scalar * ((robot.transferServoPos.getVoltage() - restPos) / 3.3) < transfer + 0.01);
    }

    public Action initShooter(int vel) {
        return new Action() {
            double velo = 0.0;
            double initPos = 0.0;
            double stamp = 0.0;
            double stamp1 = 0.0;
            double powPID = 0.0;
            double ticker = 0.0;
            double stamp2 = getRuntime();
            double currentPos = 0.0;
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp2 = getRuntime();
                }

                ticker++;
                if (ticker % 8 == 0) {
                    currentPos = (double) robot.shooter1.getCurrentPosition() / 2048;
                    stamp = getRuntime();
                    velo = -60 * ((currentPos - initPos) / (stamp - stamp1));
                    initPos = currentPos;
                    stamp1 = stamp;
                }

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

                if (vel - velo > 500) {
                    powPID = 1.0;
                }

                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);
                robot.transfer.setPower(1);

                TELE.addData("Velocity", velo);
                TELE.update();
                 if (vel - 100 < )
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

                TELE.addData("21", gpp);
                TELE.addData("22", pgp);
                TELE.addData("23", ppg);
                TELE.update();

                return !gpp || !pgp || !ppg;
            }
        };
    }

    public Action Shoot(double spindexer) {
        return new Action() {
            double transferStamp = 0.0;
            int ticker = 1;
            boolean transferIn = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spin1.setPosition(spindexer);
                robot.spin2.setPosition(1 - spindexer);
                TELE.addLine("shooting");
                TELE.update();
                if (spindexPosEqual(spindexer)) {
                    if (ticker == 1) {
                        transferStamp = getRuntime();
                        ticker++;
                    }
                    if (getRuntime() - transferStamp > waitTransfer && !transferIn) {
                        robot.transferServo.setPosition(transferServo_in);
                        transferIn = true;
                        return true;
                    } else if (getRuntime() - transferStamp > waitTransfer+waitTransferOut && transferIn){
                        robot.transferServo.setPosition(transferServo_out);
                        return false;
                    } else {
                        return true;
                    }
                } else {
                    robot.transferServo.setPosition(transferServo_out);
                    ticker = 1;
                    transferStamp = getRuntime();
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
            int b1Green = 1;
            int b1Total = 1;
            double totalStamp1 = 0.0;
            int b2Green = 1;
            int b2Total = 1;
            double totalStamp2 = 0.0;
            int b3Green = 1;
            int b3Total = 1;
            double totalStamp3 = 0.0;
            double stamp = 0.0;
            int ticker = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = getRuntime();
                    totalStamp1 = getRuntime();
                    totalStamp2 = getRuntime();
                    totalStamp3 = getRuntime();
                }
                ticker++;

                double s1D = robot.color1.getDistance(DistanceUnit.MM);
                double s2D = robot.color2.getDistance(DistanceUnit.MM);
                double s3D = robot.color3.getDistance(DistanceUnit.MM);

                if (s1D < 40) {

                    double green = robot.color1.getNormalizedColors().green;
                    double red = robot.color1.getNormalizedColors().red;
                    double blue = robot.color1.getNormalizedColors().blue;

                    double gP = green / (green + red + blue);
                    b1Total++;
                    totalStamp1 = getRuntime();
                    if (gP >= 0.35) {
                        b1Green++;
                    }
                }

                if ((b1Total > ColorCounterTotalMinCount) &&
                           ((double) b1Green / b1Total) >= ColorCounterThreshold) {
                    // Enough Time has passed and we met the threshold
                    b1 = 2;
                } else if (b1Total > ColorCounterTotalMinCount) {
                    // Enough Time passed WITHOUT meeting the threshold
                    b1 = 1;
                }

                if (s2D < 40) {

                    double green = robot.color2.getNormalizedColors().green;
                    double red = robot.color2.getNormalizedColors().red;
                    double blue = robot.color2.getNormalizedColors().blue;

                    double gP = green / (green + red + blue);

                    b2Total++;
                    totalStamp2 = getRuntime();
                    if (gP >= 0.35) {
                        b2Green++;
                    }
                }

                if ((b2Total > ColorCounterTotalMinCount) &&
                           ((double) b2Green / b2Total) >= ColorCounterThreshold) {
                    // Enough Time has passed and we met the threshold
                    b2 = 2;
                } else if (b2Total > ColorCounterTotalMinCount) {
                    // Enough Time passed WITHOUT meeting the threshold
                    b2 = 1;
                }

                if (s3D < 40) {

                    double green = robot.color3.getNormalizedColors().green;
                    double red = robot.color3.getNormalizedColors().red;
                    double blue = robot.color3.getNormalizedColors().blue;

                    double gP = green / (green + red + blue);

                    b3Total++;
                    totalStamp3 = getRuntime();

                    if (gP >= 0.35) {
                        b3Green++;
                    }
                }

                if ((b3Total > ColorCounterTotalMinCount) &&
                           ((double) b3Green / b3Total) >= ColorCounterThreshold) {
                    // Enough Time has passed and we met the threshold
                    b3 = 2;
                } else if (b3Total > ColorCounterTotalMinCount) {
                    // Enough Time passed WITHOUT meeting the threshold
                    b3 = 1;
                }

                TELE.addLine("Detecting");
                TELE.addData("Distance 1", s1D);
                TELE.addData("Distance 2", s2D);
                TELE.addData("Distance 3", s3D);
                TELE.addData("B1", b1);
                TELE.addData("B2", b2);
                TELE.addData("B3", b3);
                TELE.update();

                return (b1 + b2 + b3 < 4) || getRuntime() - stamp < 4.0;
            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        drive = new MecanumDrive(hardwareMap, new Pose2d(
                0, 0, 0
        ));

        aprilTag = new AprilTagWebcam();

        TrajectoryActionBuilder shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1);

        TrajectoryActionBuilder pickup1 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .strafeToLinearHeading(new Vector2d(x2a, y2a), h2a)
                .strafeToLinearHeading(new Vector2d(x2b, y2b), h2b);

        TrajectoryActionBuilder shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, h2b))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1);

        TrajectoryActionBuilder pickup2 = drive.actionBuilder(new Pose2d(x1, y1, h1))

                .strafeToLinearHeading(new Vector2d(x3a, y3a), h3a)

                .strafeToLinearHeading(new Vector2d(x3b, y3b), h3b);

        TrajectoryActionBuilder shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, h3b))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1);

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .strafeToLinearHeading(new Vector2d(x1, y1 + 30), h1);

        aprilTag.init(robot, TELE);

        while (opModeInInit()) {

            if (gamepad2.dpadUpWasPressed()) {
                hoodDefault -= 0.01;
            }

            if (gamepad2.dpadDownWasPressed()) {
                hoodDefault += 0.01;
            }

            robot.hood.setPosition(hoodDefault);

            robot.turr1.setPosition(turret_detect);
            robot.turr2.setPosition(1 - turret_detect);

            robot.transferServo.setPosition(transferServo_out);

            robot.spin1.setPosition(spindexer_intakePos1);
            robot.spin2.setPosition(1 - spindexer_intakePos1);

            aprilTag.update();

            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            robot.hood.setPosition(hoodDefault);

            Actions.runBlocking(
                    new ParallelAction(
                            shoot0.build(),
                            initShooter(AUTO_CLOSE_VEL),
                            Obelisk()
                    )
            );

            robot.turr1.setPosition(turret_red);
            robot.turr2.setPosition(1 - turret_red);

            shootingSequence();

            Actions.runBlocking(
                    new ParallelAction(
                            pickup1.build(),
                            intake(intake1Time)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            shoot1.build(),
                            ColorDetect()
                    )
            );

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
                            ColorDetect()
                    )
            );

            shootingSequence();

            Actions.runBlocking(
                    park.build()
            );

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            TELE.addLine("finished");

            TELE.update();

            sleep(2000);

        }

    }

    public void shootingSequence() {
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
                        Shoot(spindexer_outtakeBall1),
                        Shoot(spindexer_outtakeBall2),
                        Shoot(spindexer_outtakeBall3)
                )
        );
    }

    public void sequence2() {
        Actions.runBlocking(
                new SequentialAction(
                        Shoot(spindexer_outtakeBall1),
                        Shoot(spindexer_outtakeBall3),
                        Shoot(spindexer_outtakeBall2)
                )
        );
    }

    public void sequence3() {
        Actions.runBlocking(
                new SequentialAction(
                        Shoot(spindexer_outtakeBall2),
                        Shoot(spindexer_outtakeBall1),
                        Shoot(spindexer_outtakeBall3)
                )
        );
    }

    public void sequence4() {
        Actions.runBlocking(
                new SequentialAction(
                        Shoot(spindexer_outtakeBall2),
                        Shoot(spindexer_outtakeBall3),
                        Shoot(spindexer_outtakeBall1)
                )
        );
    }

    public void sequence5() {
        Actions.runBlocking(
                new SequentialAction(
                        Shoot(spindexer_outtakeBall3),
                        Shoot(spindexer_outtakeBall1),
                        Shoot(spindexer_outtakeBall2)
                )
        );
    }

    public void sequence6() {
        Actions.runBlocking(
                new SequentialAction(
                        Shoot(spindexer_outtakeBall3),
                        Shoot(spindexer_outtakeBall2),
                        Shoot(spindexer_outtakeBall1)
                )
        );
    }
}
