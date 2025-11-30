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
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@Autonomous
public class redDaniel extends LinearOpMode {

    Robot robot;

    MultipleTelemetry TELE;

    MecanumDrive drive;

    AprilTag aprilTag;

    int b1 = 0; // 0 = no ball, 1 = green, 2 = purple

    int b2 = 0;// 0 = no ball, 1 = green, 2 = purple

    int b3 = 0;// 0 = no ball, 1 = green, 2 = purple
    // TODO: change this velocity PID
    public Action initShooter(int velocity){
        return new Action(){
            double velo = 0.0;
            double initPos = 0.0;
            double stamp = 0.0;
            double powPID = 0.0;
            double ticker = 0.0;
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                velo = -60 * ((((double) robot.shooter1.getCurrentPosition() / 2048) - initPos) / (getRuntime() - stamp));
                stamp = getRuntime();
                initPos = (double) robot.shooter1.getCurrentPosition() / 2048;
                if (Math.abs(velocity - velo) > initTolerance) {
                    powPID = (double) velocity / maxVel;
                    ticker = getRuntime();
                } else if (velocity - velTolerance > velo) {
                    powPID = powPID + 0.0001;
                    ticker = getRuntime();
                } else if (velocity + velTolerance < velo) {
                    powPID = powPID - 0.0001;
                    ticker  = getRuntime();
                }
                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);
                robot.transfer.setPower((powPID / 4) + 0.75);

                return getRuntime() - ticker < 0.5;
            }
        };
    }

    public void Obelisk (){
        // TODO: write the code to detect order
    }

    public Action Shoot(double spindexer){
        return new Action() {
            boolean transfer = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spin1.setPosition(spindexer);
                robot.spin2.setPosition(1-spindexer);
                if (scalar*((robot.spin1Pos.getVoltage() - restPos) / 3.3) < spindexer + 0.01 && scalar*((robot.spin1Pos.getVoltage() - restPos) / 3.3) > spindexer - 0.01){
                    robot.transferServo.setPosition(transferServo_in);
                    transfer = true;
                }
                if (scalar*((robot.transferServoPos.getVoltage() - restPos) / 3.3) < transferServo_in + 0.01 && scalar*((robot.transferServoPos.getVoltage() - restPos) / 3.3) > transferServo_in - 0.01 && transfer){
                    robot.transferServo.setPosition(transferServo_out);
                    return false;
                }
                return true;
            }
        };
    }

    public Action intake (){
        return new Action() {
            double position = 0.0;
            final double intakeTime = 4.0; // TODO: change this so it serves as a backup
            final double stamp = getRuntime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if ((getRuntime() % 0.3) >0.15) {
                    position = spindexer_intakePos1 + 0.02;
                } else {
                    position = spindexer_intakePos1 - 0.02;
                }
                robot.spin1.setPosition(position);
                robot.spin2.setPosition(1-position);

                robot.intake.setPower(1);

                return !(robot.pin1.getState() && robot.pin3.getState() && robot.pin5.getState()) || getRuntime() - stamp > intakeTime;
            }
        };
    }

    public Action ColorDetect (){
        return new Action() {
            int t1 = 0;
            int t2 = 0;
            int t3 = 0;
            int tP1 = 0;
            int tP2 = 0;
            int tP3 = 0;
            final double stamp = getRuntime();
            final double detectTime = 3.0;
            double position = 0.0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if ((getRuntime() % 0.3) >0.15) {
                    position = spindexer_intakePos1 + 0.02;
                } else {
                    position = spindexer_intakePos1 - 0.02;
                }
                robot.spin1.setPosition(position);
                robot.spin2.setPosition(1-position);
                if (robot.pin1.getState()) {
                    t1 += 1;
                    if (robot.pin0.getState()){
                        tP1 += 1;
                    }
                }
                if (robot.pin3.getState()) {
                    t2 += 1;
                    if (robot.pin0.getState()){
                        tP2 += 1;
                    }
                }
                if (robot.pin5.getState()) {
                    t3 += 1;
                    if (robot.pin0.getState()){
                        tP3 += 1;
                    }
                }
                if (t1 > 20){
                    if (tP1 > 20){
                        b1 = 2;
                    } else {
                        b1 = 1;
                    }
                }
                if (t2 > 20){
                    if (tP2 > 20){
                        b2 = 2;
                    } else {
                        b2 = 1;
                    }
                }
                if (t3 > 20){
                    if (tP3 > 20){
                        b3 = 2;
                    } else {
                        b3 = 1;
                    }
                }
                return !(b1 + b2 + b3 >= 5) || (getRuntime() - stamp < detectTime);

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

        aprilTag = new AprilTag(robot, TELE);

        TrajectoryActionBuilder shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1);

        TrajectoryActionBuilder pickup1 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .turnTo(Math.toRadians(h2))
                .strafeToLinearHeading(new Vector2d(x2, y2), h2);

        TrajectoryActionBuilder shoot1 = drive.actionBuilder(new Pose2d(x2, y2, h2))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1);

        TrajectoryActionBuilder pickup2 = drive.actionBuilder(new Pose2d(x1, y1, h1))

                .strafeToLinearHeading(new Vector2d(x2_b, y2_b), h2_b)

                .strafeToLinearHeading(new Vector2d(x3, y3), h3);

        TrajectoryActionBuilder shoot2 = drive.actionBuilder(new Pose2d(x3, y3, h3))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1);

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .strafeToLinearHeading(new Vector2d(x1, y1 + 30), h1);

        while (opModeInInit()) {

            if (gamepad2.dpadUpWasPressed()) {
                hoodDefault -= 0.01;
            }

            if (gamepad2.dpadDownWasPressed()) {
                hoodDefault += 0.01;
            }

            robot.hood.setPosition(hoodDefault);

            robot.turr1.setPosition(turret_red);
            robot.turr2.setPosition(1 - turret_red);

            robot.transferServo.setPosition(transferServo_out);

            aprilTag.initTelemetry();

            aprilTag.update();

            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            robot.hood.setPosition(hoodDefault);

            Actions.runBlocking(
                    new ParallelAction(
                            shoot0.build()
                    )
            );



            Actions.runBlocking(
                    pickup1.build()
            );

            Actions.runBlocking(
                    shoot1.build()
            );


            Actions.runBlocking(
                    pickup2.build()
            );

            Actions.runBlocking(
                    shoot2.build()
            );


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
}
