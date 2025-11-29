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
