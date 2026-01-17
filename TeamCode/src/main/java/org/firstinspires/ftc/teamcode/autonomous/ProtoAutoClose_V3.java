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
public class ProtoAutoClose_V3 extends LinearOpMode {
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
                        double turretPID = servo.setTurrPos(turret_redClose, robot.turr1Pos.getCurrentPosition());
                        robot.turr1.setPower(turretPID);
                        robot.turr2.setPower(-turretPID);
                        return !servo.turretEqual(turret_redClose, robot.turr1Pos.getCurrentPosition());

                    } else {
                        robot.limelight.pipelineSwitch(2);
                        double turretPID = servo.setTurrPos(turret_blueClose, robot.turr1Pos.getCurrentPosition());
                        robot.turr1.setPower(turretPID);
                        robot.turr2.setPower(-turretPID);
                        return !servo.turretEqual(turret_blueClose, robot.turr1Pos.getCurrentPosition());
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
            int ticker = 1;
            double initPos = 0.0;
            double finalPos = 0.0;
            boolean zeroNeeded = false;
            boolean zeroPassed = false;
            double currentPos = 0.0;
            double prevPos = 0.0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                TELE.addData("Velocity", velo);
                TELE.addLine("shooting");
                TELE.update();

                powPID = flywheel.manageFlywheel(vel, robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                velo = flywheel.getVelo(robot.shooter1.getCurrentPosition(), robot.shooter2.getCurrentPosition());
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);

                robot.turr1.setPower(holdTurrPow);
                robot.turr2.setPower(holdTurrPow);

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                if (ticker == 1){
                    robot.transferServo.setPosition(transferServo_in);
                    initPos = servo.getSpinPos(robot.spin1Pos.getVoltage());

                    finalPos = initPos + 0.6;

                    if (finalPos > 1.0){
                        finalPos = finalPos - 1;
                        zeroNeeded = true;
                    } else if (finalPos > 0.95){
                        finalPos = 0.0;
                        zeroNeeded = true;
                    }
                    currentPos = initPos;
                }
                ticker++;

                if (ticker > 16){
                    robot.spin1.setPower(0.08);
                    robot.spin2.setPower(-0.08);
                }

                prevPos = currentPos;
                currentPos = servo.getSpinPos(robot.spin1Pos.getVoltage());
                if (zeroNeeded){
                    if (currentPos - prevPos < -0.5){
                        zeroPassed = true;
                    }
                    if (zeroPassed){
                        if (currentPos > finalPos){
                            robot.spin1.setPower(0);
                            robot.spin2.setPower(0);
                            return false;
                        } else {
                            return true;
                        }
                    } else {
                        return true;
                    }
                } else {
                    if (currentPos > finalPos){
                        robot.spin1.setPower(0);
                        robot.spin2.setPower(0);
                        return false;
                    } else {
                        return true;
                    }
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
                        if (servo.spinEqual(spindexer_intakePos1,robot.spin1Pos.getVoltage())){
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

        servo = new Servos();

        robot.limelight.pipelineSwitch(1);
        robot.limelight.start();

        TrajectoryActionBuilder shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

        TrajectoryActionBuilder pickup1 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                .strafeToLinearHeading(new Vector2d(bx2a, by2a), bh2a)
                .strafeToLinearHeading(new Vector2d(bx2b, by2b), bh2b);

        TrajectoryActionBuilder shoot1 = drive.actionBuilder(new Pose2d(bx2b, by2b, bh2b))
                .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

        TrajectoryActionBuilder pickup2 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                .strafeToLinearHeading(new Vector2d(bx3a, by3a), bh3a)
                .strafeToLinearHeading(new Vector2d(bx3b, by3b), bh3b);

        TrajectoryActionBuilder shoot2 = drive.actionBuilder(new Pose2d(bx3b, by3b, bh3b))
                .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

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

            if (redAlliance){
                shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

                pickup1 = drive.actionBuilder(new Pose2d(rx1, ry1, rh1))
                        .strafeToLinearHeading(new Vector2d(rx2a, ry2a), rh2a)
                        .strafeToLinearHeading(new Vector2d(rx2b, ry2b), rh2b);

                shoot1 = drive.actionBuilder(new Pose2d(rx2b, ry2b, rh2b))
                        .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);

                pickup2 = drive.actionBuilder(new Pose2d(rx1, ry1, rh1))
                        .strafeToLinearHeading(new Vector2d(rx3a, ry3a), rh3a)
                        .strafeToLinearHeading(new Vector2d(rx3b, ry3b), rh3b);

                shoot2 = drive.actionBuilder(new Pose2d(rx3b, ry3b, rh3b))
                        .strafeToLinearHeading(new Vector2d(rx1, ry1), rh1);
            } else {
                shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

                pickup1 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                        .strafeToLinearHeading(new Vector2d(bx2a, by2a), bh2a)
                        .strafeToLinearHeading(new Vector2d(bx2b, by2b), bh2b);

                shoot1 = drive.actionBuilder(new Pose2d(bx2b, by2b, bh2b))
                        .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);

                pickup2 = drive.actionBuilder(new Pose2d(bx1, by1, bh1))
                        .strafeToLinearHeading(new Vector2d(bx3a, by3a), bh3a)
                        .strafeToLinearHeading(new Vector2d(bx3b, by3b), bh3b);

                shoot2 = drive.actionBuilder(new Pose2d(bx3b, by3b, bh3b))
                        .strafeToLinearHeading(new Vector2d(bx1, by1), bh1);
            }

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
                    new ParallelAction(
                            shoot1.build()
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
                            shoot2.build()
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
        double turretPos = servo.getTurrPos(robot.turr1Pos.getCurrentPosition()) - (bearing / 1300);
        double turretPID = servo.setTurrPos(turretPos, robot.turr1Pos.getCurrentPosition());
        robot.turr1.setPower(turretPID);
        robot.turr2.setPower(-turretPID);
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