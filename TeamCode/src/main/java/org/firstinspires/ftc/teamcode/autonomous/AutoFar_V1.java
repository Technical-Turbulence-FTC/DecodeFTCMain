package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.*;
import static org.firstinspires.ftc.teamcode.constants.Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;
import static org.firstinspires.ftc.teamcode.constants.ShooterVars.*;

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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.FlywheelV2;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;

import java.util.List;

@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class AutoFar_V1 extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    MecanumDrive drive;
    FlywheelV2 flywheel;
    Servos servo;

    double velo = 0.0;
    public static double intake1Time = 2.7;
    public static double intake2Time = 3.0;
    public static double colorDetect = 3.0;
    boolean gpp = false;
    boolean pgp = false;
    boolean ppg = false;
    double powPID = 0.0;
    double bearing = 0.0;
    int b1 = 0; // 0 = no ball, 1 = green, 2 = purple
    int b2 = 0;// 0 = no ball, 1 = green, 2 = purple
    int b3 = 0;// 0 = no ball, 1 = green, 2 = purple
    public static double holdTurrPow = 0.01; // power to hold turret in place

    public Action initShooter(int vel) {
        return new Action() {
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                powPID = flywheel.manageFlywheel(vel, robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                velo = flywheel.getVelo(robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);

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

                if (id == 21){
                    gpp = true;
                } else if (id == 22){
                    pgp = true;
                } else if (id == 23){
                    ppg = true;
                }

                TELE.addData("Velocity", velo);
                TELE.addData("21", gpp);
                TELE.addData("22", pgp);
                TELE.addData("23", ppg);
                TELE.update();

                if (gpp || pgp || ppg) {
                    if (redAlliance){
                        robot.limelight.pipelineSwitch(3);
                        double turretPID = servo.setTurrPos(turret_redFar, robot.turr1Pos.getCurrentPosition());
                        robot.turr1.setPower(turretPID);
                        robot.turr2.setPower(-turretPID);
                        return !servo.turretEqual(turret_redFar, robot.turr1Pos.getCurrentPosition());

                    } else {
                        robot.limelight.pipelineSwitch(2);
                        double turretPID = servo.setTurrPos(turret_blueFar, robot.turr1Pos.getCurrentPosition());
                        robot.turr1.setPower(turretPID);
                        robot.turr2.setPower(-turretPID);
                        return !servo.turretEqual(turret_blueFar, robot.turr1Pos.getCurrentPosition());
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
                powPID = flywheel.manageFlywheel(vel, robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                velo = flywheel.getVelo(robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);

                spinPID = servo.setSpinPos(spindexer, robot.spin1Pos.getVoltage());
                robot.spin1.setPower(spinPID);
                robot.spin2.setPower(-spinPID);
                TELE.addData("Velocity", velo);
                TELE.addLine("spindex");
                TELE.update();

                drive.updatePoseEstimate();
                teleStart = drive.localizer.getPose();

                if (servo.spinEqual(spindexer, robot.spin1Pos.getVoltage())){
                    robot.spin1.setPower(0);
                    robot.spin2.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        };
    }

    public Action Shoot(int vel) {
        return new Action() {
            double transferStamp = 0.0;
            int ticker = 1;
            boolean transferIn = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                TELE.addData("Velocity", velo);
                TELE.addLine("shooting");
                TELE.update();

                powPID = flywheel.manageFlywheel(vel, robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                velo = flywheel.getVelo(robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);

                drive.updatePoseEstimate();
                detectTag();

                teleStart = drive.localizer.getPose();

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
                } else if (getRuntime() - transferStamp > waitTransfer + waitTransferOut && transferIn) {
                    robot.transferServo.setPosition(transferServo_out);
                    robot.turr1.setPower(holdTurrPow);
                    robot.turr2.setPower(holdTurrPow);
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
            double position = spindexer_intakePos1;
            double stamp = 0.0;
            int ticker = 0;
            double pow = 1.0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = getRuntime();
                }
                ticker++;

                robot.intake.setPower(pow);

                double s1D = robot.color1.getDistance(DistanceUnit.MM);
                double s2D = robot.color2.getDistance(DistanceUnit.MM);
                double s3D = robot.color3.getDistance(DistanceUnit.MM);

                if (!servo.spinEqual(position, robot.spin1Pos.getVoltage())){
                    double spinPID = servo.setSpinPos(position, robot.spin1Pos.getVoltage());
                    robot.spin1.setPower(spinPID);
                    robot.spin2.setPower(-spinPID);
                }

                if (s1D < 43 && servo.spinEqual(position, robot.spin1Pos.getVoltage()) && getRuntime() - stamp > 0.5){
                    if (s2D > 60){
                        if (servo.spinEqual(spindexer_intakePos1, robot.spin1Pos.getVoltage())){
                            position = spindexer_intakePos2;
                        } else if (servo.spinEqual(spindexer_intakePos2, robot.spin1Pos.getVoltage())){
                            position = spindexer_intakePos3;
                        } else if (servo.spinEqual(spindexer_intakePos3, robot.spin1Pos.getVoltage())){
                            position = spindexer_intakePos1;
                        }
                    } else if (s3D > 33){
                        if (servo.spinEqual(spindexer_intakePos1, robot.spin1Pos.getVoltage())){
                            position = spindexer_intakePos3;
                        } else if (servo.spinEqual(spindexer_intakePos2, robot.spin1Pos.getVoltage())){
                            position = spindexer_intakePos1;
                        } else if (servo.spinEqual(spindexer_intakePos3, robot.spin1Pos.getVoltage())){
                            position = spindexer_intakePos2;
                        }
                    }
                    stamp = getRuntime();
                }

                TELE.addData("Velocity", velo);
                TELE.addLine("Intaking");
                TELE.update();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                robot.intake.setPower(1);
                if ((s1D < 43.0 && s2D < 60.0 && s3D < 33.0) || getRuntime() - stamp > intakeTime) {
                    robot.spin1.setPower(0);
                    robot.spin2.setPower(0);
                    if (getRuntime() - stamp - intakeTime < 1){
                        pow = -2*(getRuntime() - stamp - intakeTime);
                        return true;
                    } else {
                        robot.intake.setPower(0);
                        return false;
                    }
                } else {
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

                powPID = flywheel.manageFlywheel(vel, robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                velo = flywheel.getVelo(robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);

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

        flywheel = new FlywheelV2();

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        drive = new MecanumDrive(hardwareMap, new Pose2d(
                0, 0, 0
        ));

        robot.limelight.pipelineSwitch(1);
        robot.limelight.start();

        //TODO: add positions to develop auto
        
        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeToLinearHeading(new Vector2d(rfx1, rfy1), rfh1);

        while (opModeInInit()) {

            if (gamepad2.dpadUpWasPressed()) {
                hoodAuto -= 0.01;
            }

            if (gamepad2.dpadDownWasPressed()) {
                hoodAuto += 0.01;
            }

            if (gamepad2.crossWasPressed()){
                redAlliance = !redAlliance;
            }

            double turrPID;

            if (redAlliance){
                turrPID = servo.setTurrPos(turret_detectRedClose, robot.turr1Pos.getCurrentPosition());
            } else {
                turrPID = servo.setTurrPos(turret_detectBlueClose, robot.turr1Pos.getCurrentPosition());
            }

            robot.turr1.setPower(turrPID);
            robot.turr2.setPower(-turrPID);

            robot.hood.setPosition(hoodAutoFar);

            robot.transferServo.setPosition(transferServo_out);

            TELE.addData("Velocity", velo);
            TELE.addData("Turret Pos", servo.getTurrPos(robot.turr1Pos.getCurrentPosition()));
            TELE.addData("Spin Pos", servo.getSpinPos(robot.spin1Pos.getVoltage()));
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            Actions.runBlocking(
                    new ParallelAction(
                            initShooter(AUTO_FAR_VEL),
                            Obelisk()
                    )
            );
            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            robot.transfer.setPower(1);

            shootingSequence();

            robot.transfer.setPower(0);

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();
            
            Actions.runBlocking(park.build());

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
        double turretPos = servo.getTurrPos(robot.turr1Pos.getCurrentPosition()) - (bearing / 1300);
        double turretPID = servo.setTurrPos(turretPos, robot.turr1Pos.getCurrentPosition());
        robot.turr1.setPower(turretPID);
        robot.turr2.setPower(-turretPID);
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
                        spindex(spindexer_outtakeBall1, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall2, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall3, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL)
                )
        );
    }

    public void sequence2() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall1, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall3, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall2, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL)
                )
        );
    }

    public void sequence3() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall2, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall1, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall3, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL)
                )
        );
    }

    public void sequence4() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall2, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall3, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall1, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL)
                )
        );
    }

    public void sequence5() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall3, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall1, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall2, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL)
                )
        );
    }

    public void sequence6() {
        Actions.runBlocking(
                new SequentialAction(
                        spindex(spindexer_outtakeBall3, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall2, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL),
                        spindex(spindexer_outtakeBall1, AUTO_FAR_VEL),
                        Shoot(AUTO_FAR_VEL)
                )
        );
    }
}