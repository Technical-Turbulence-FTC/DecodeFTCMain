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
import org.firstinspires.ftc.teamcode.utils.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@Autonomous
public class redDaniel extends LinearOpMode {

    Robot robot;

    MultipleTelemetry TELE;

    MecanumDrive drive;

    AprilTagWebcam aprilTag;

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
    final double ColorCounterThreshold  = 0.65;

    boolean spindexPosEqual (double spindexer) {
        return (scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) > spindexer - 0.01 &&
                scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) < spindexer + 0.01);
    }

    boolean transferPosEqual (double transfer) {
        return (scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) > transfer - 0.01 &&
                scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3) < transfer + 0.01);
    }
    public Action initShooter(int vel){
        return new Action(){
            double velo = 0.0;
            double initPos = 0.0;
            double stamp = 0.0;
            double stamp1 = 0.0;
            double powPID = 0.0;
            double ticker = 0.0;
            final double time = getRuntime();
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                ticker ++;

                double currentPos;
                if (ticker % 8 == 0) {
                    currentPos = (double) robot.shooter1.getCurrentPosition() / 2048;
                    stamp = getRuntime();
                    velo = -60 * ((currentPos - initPos) / (stamp - stamp1));
                    initPos = currentPos;
                    stamp1 = stamp;
                }

                double feed = (double) vel / maxVel;        // Example: vel=2500 → feed=0.5

                if (vel > 500) {
                    feed = Math.log((668.39 / (vel + 591.96)) - 0.116) / -4.18;
                }

                // --- PROPORTIONAL CORRECTION ---
                double error = vel - velo;
                double correction = kP * error;

                // limit how fast power changes (prevents oscillation)
                correction = Math.max(-maxStep, Math.min(maxStep, correction));

                // --- FINAL MOTOR POWER ---
                powPID = feed + correction;

                // clamp to allowed range
                powPID = Math.max(0, Math.min(1, powPID));

                if (vel - velo > 1000) {
                    powPID = 1;
                } else if (velo - vel > 1000) {
                    powPID = 0;
                }

                robot.shooter1.setPower(powPID);
                robot.shooter2.setPower(powPID);

                return !(vel - 100 < velo && vel + 100 > velo) || (getRuntime() - time > 4.0);
            }
        };
    }

    public Action Obelisk (){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (aprilTag.getTagById(21) != null){
                    gpp = true;
                } else if (aprilTag.getTagById(22) != null){
                    pgp = true;
                } else if (aprilTag.getTagById(23) != null){
                    ppg = true;
                }
                return !gpp && !pgp && !ppg;
            }
        };
    }

    public Action Shoot(double spindexer){
        return new Action() {
            double transferStamp = 0.0;
            int ticker = 1;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.spin1.setPosition(spindexer);
                robot.spin2.setPosition(1 - spindexer);
                if (spindexPosEqual(spindexer)){
                    if (ticker == 1){
                        transferStamp = getRuntime();
                        ticker ++;
                    }
                    if (getRuntime() - transferStamp > waitTransfer) {
                        robot.transferServo.setPosition(transferServo_in);
                    } else {
                        robot.transferServo.setPosition(transferServo_out);
                    }
                } else {
                    robot.transferServo.setPosition(transferServo_out);
                    ticker = 1;
                    transferStamp = getRuntime();
                }

                return !(transferPosEqual(transferServo_in));
            }
        };
    }

    public Action transferOut(){
        return new Action() {
            final double transfer = getRuntime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (getRuntime() - transfer > 0.2){
                    robot.transferServo.setPosition(transferServo_out);
                    return false;
                } else {
                    return true;
                }
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
                // TODO: change return statement
                return !(robot.pin1.getState() && robot.pin3.getState() && robot.pin5.getState()) || getRuntime() - stamp < intakeTime;
            }
        };
    }
    //TODO: use i2c code to write this
    public Action ColorDetect (){
        return new Action() {
            int b1Green = 1;
            int b1Total = 1;
            double totalStamp1 = getRuntime();
            int b2Green = 1;
            int b2Total = 1;
            double totalStamp2 = getRuntime();
            int b3Green = 1;
            int b3Total = 1;
            double totalStamp3 = getRuntime();
            double stamp = getRuntime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (robot.pin1.getState()){
                    if (robot.pin0.getState()){
                        b1Green++;
                    }
                    b1Total++;
                    totalStamp1 = getRuntime();
                }
                if (getRuntime() - totalStamp1 > ColorCounterResetDelay) {
                    // Too Much time has passed without detecting ball
                    b1 = 0;
                    b1Total = 1;
                    b1Green = 1;
                }else if ((b1Total > ColorCounterTotalMinCount) && ((double) b1Green / b1Total) >= ColorCounterThreshold){
                    // Enough Time has passed and we met the threshold
                    b1 = 1;
                }else if (b1Total > ColorCounterTotalMinCount) {
                    // Enough Time passed WITHOUT meeting the threshold
                    b1 = 2;
                }

                if (robot.pin3.getState()){
                    if (robot.pin2.getState()){
                        b2Green++;
                    }
                    b2Total++;
                    totalStamp2 = getRuntime();
                }
                if (getRuntime() - totalStamp2 > ColorCounterResetDelay) {
                    // Too Much time has passed without detecting ball
                    b2 = 0;
                    b2Total = 1;
                    b2Green = 1;
                }else if ((b2Total > ColorCounterTotalMinCount) && ((double) b2Green / b2Total) >= ColorCounterThreshold){
                    // Enough Time has passed and we met the threshold
                    b2 = 1;
                }else if (b2Total > ColorCounterTotalMinCount) {
                    // Enough Time passed WITHOUT meeting the threshold
                    b2 = 2;
                }

                if (robot.pin5.getState()){
                    if (robot.pin4.getState()){
                        b3Green++;
                    }
                    b3Total++;
                    totalStamp3 = getRuntime();
                }
                if (getRuntime() - totalStamp3 > ColorCounterResetDelay) {
                    // Too Much time has passed without detecting ball
                    b3 = 0;
                    b3Total = 1;
                    b3Green = 1;
                }else if ((b3Total > ColorCounterTotalMinCount) && ((double) b3Green / b3Total) >= ColorCounterThreshold){
                    // Enough Time has passed and we met the threshold
                    b3 = 1;
                }else if (b3Total > ColorCounterTotalMinCount) {
                    // Enough Time passed WITHOUT meeting the threshold
                    b3 = 2;
                }

                return !(b1 + b2 + b3 >= 5) || getRuntime() - stamp < 3.0;
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

        aprilTag.init(robot, TELE);

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
