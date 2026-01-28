package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.*;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bh2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bh2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bh2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bt2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bt2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bt2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bx1;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bx2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bx2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.bx2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.by1;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.by2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.by2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.by2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rh1;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rh2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rh2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rh2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rt2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rt2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rt2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rx1;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rx2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rx2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.rx2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.ry1;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.ry2a;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.ry2b;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.ry2c;
import static org.firstinspires.ftc.teamcode.constants.Poses_V2.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Autonomous(preselectTeleOp = "TeleopV3")
@Config
public class Auto_LT extends LinearOpMode {

    public int motif = 0;

    public static double shoot0Vel = 2300, shoot0Hood = 0.93;
    public static double autoSpinStartPos = 0.2;
    public static double shoot0SpinSpeedIncrease = 0.014;
    public static double obeliskTurrPos = 0.53;

    public static double shoot1Turr = 0.57;

    public static double shoot0XTolerance = 1.0;
    public static double shoot0Time = 1.8;

    public static double intake1Time = 5.0;

    public static double flywheel0Time = 3.5;
    public static double pickup1Speed = 20.0;

    // ---- SECOND SHOT / PICKUP ----
    public static double shoot1Vel = 2300;
    public static double shoot1Hood = 0.93;

    // ---- PICKUP TIMING ----
    public static double pickup1Time = 3.0;

    // ---- PICKUP POSITION TOLERANCES ----
    public static double pickup1XTolerance = 2.0;
    public static double pickup1YTolerance = 2.0;

    // ---- OBELISK DETECTION ----
    public static double obelisk1Time = 1.5;
    public static double obelisk1XTolerance = 2.0;
    public static double obelisk1YTolerance = 2.0;

    public static double shoot1ToleranceX = 2.0;
    public static double shoot1ToleranceY = 2.0;
    public static double shoot1Time = 2.0;


    Robot robot;
    MultipleTelemetry TELE;
    MecanumDrive drive;
    Servos servos;
    Spindexer spindexer;
    Flywheel flywheel;
    Turret turret;
    private double x1, y1, h1;

    private double x2a, y2a, h2a, t2a;

    private double x2b, y2b, h2b, t2b;
    private double x2c, y2c, h2c, t2c;
    
    private double xShoot, yShoot, hShoot;

    public Action prepareShootAll(double time) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;

                robot.turr1.setPosition(shoot1Turr);
                robot.turr1.setPosition(1-shoot1Turr);


                robot.spin1.setPosition(autoSpinStartPos);
                robot.spin2.setPosition(1 - autoSpinStartPos);

                robot.transferServo.setPosition(transferServo_out);

                robot.intake.setPower(-((System.currentTimeMillis() - stamp))/1000);

                return (System.currentTimeMillis() - stamp) < (time * 1000);


            }
        };
    }

    public Action shootAll(int vel, double shootTime) {
        return new Action() {
            int ticker = 1;

            double stamp = 0.0;

            double velo = vel;

            int shooterTicker = 0;

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

                if (getRuntime() - stamp < shootTime) {

                    if (shooterTicker == 0 && !servos.spinEqual(autoSpinStartPos)) {
                        robot.spin1.setPosition(autoSpinStartPos);
                        robot.spin2.setPosition(1 - autoSpinStartPos);
                    } else {
                        robot.transferServo.setPosition(transferServo_in);
                        shooterTicker++;
                        double prevSpinPos = robot.spin1.getPosition();
                        robot.spin1.setPosition(prevSpinPos + shoot0SpinSpeedIncrease);
                        robot.spin2.setPosition(1 - prevSpinPos - shoot0SpinSpeedIncrease);
                    }

                    return true;

                } else {
                    robot.transferServo.setPosition(transferServo_out);
                    //spindexPos = spindexer_intakePos1;

                    spindexer.resetSpindexer();
                    spindexer.processIntake();

                    return false;

                }

            }
        };
    }

    public Action intake(double intakeTime) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;

                spindexer.processIntake();
                robot.intake.setPower(1);

                motif = turret.detectObelisk();

                return (System.currentTimeMillis() - stamp) < (intakeTime * 1000);


            }
        };
    }


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

                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }

                ticker++;

                motif = turret.detectObelisk();

                robot.turr1.setPosition(turrPos);
                robot.turr2.setPosition(1 - turrPos);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || xDone || yDone;

                return !shouldFinish;

            }
        };
    }

    public Action manageFlywheel(
            double vel,
            double hoodPos,
            double time,
            double posX,
            double posY,
            double posXTolerance,
            double posYTolerance
    ) {

        boolean timeFallback = (time != 0.501);
        boolean posXFallback = (posX != 0.501);
        boolean posYFallback = (posY != 0.501);

        return new Action() {

            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }

                ticker++;

                flywheel.manageFlywheel(vel);
                robot.hood.setPosition(hoodPos);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || xDone || yDone;

                return !shouldFinish;

            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap.get(Servo.class, "light").setPosition(0);

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        flywheel = new Flywheel(hardwareMap);

        Targeting targeting = new Targeting();
        Targeting.Settings targetingSettings = new Targeting.Settings(0.0, 0.0);

        spindexer = new Spindexer(hardwareMap);

        servos = new Servos(hardwareMap);

        robot.limelight.pipelineSwitch(1);

        turret = new Turret(robot, TELE, robot.limelight);

        turret.manualSetTurret(0.4);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        TrajectoryActionBuilder shoot0;

        TrajectoryActionBuilder pickup1;
        TrajectoryActionBuilder shoot1;

        robot.spin1.setPosition(autoSpinStartPos);
        robot.spin2.setPosition(1 - autoSpinStartPos);

        robot.transferServo.setPosition(transferServo_out);

        robot.light.setPosition(1);

        while (opModeInInit()) {

            robot.hood.setPosition(shoot0Hood);

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;
            }

            if (redAlliance) {
                robot.light.setPosition(0.28);

                // ---- FIRST SHOT ----
                x1 = rx1;
                y1 = ry1;
                h1 = rh1;

                // ---- PICKUP PATH ----
                x2a = rx2a;
                y2a = ry2a;
                h2a = rh2a;
                t2a = rt2a;
                x2b = rx2b;
                y2b = ry2b;
                h2b = rh2b;
                t2b = rt2b;
                x2c = rx2c;
                y2c = ry2c;
                h2c = rh2c;
                t2c = rt2c;
                
                xShoot = rShootX;
                yShoot = rShootY;
                hShoot = rShootH;

            } else {
                robot.light.setPosition(0.6);

                // ---- FIRST SHOT ----
                x1 = bx1;
                y1 = by1;
                h1 = bh1;

                // ---- PICKUP PATH ----
                x2a = bx2a;
                y2a = by2a;
                h2a = bh2a;
                t2a = bt2a;
                x2b = bx2b;
                y2b = by2b;
                h2b = bh2b;
                t2b = bt2b;
                x2c = bx2c;
                y2c = by2c;
                h2c = bh2c;
                t2c = bt2c;

                xShoot = bShootX;
                yShoot = bShootY;
                hShoot = bShootH;
            }
            shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .strafeToLinearHeading(new Vector2d(x1, y1), h1);

            pickup1 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                    .strafeToLinearHeading(new Vector2d(x2a, y2a), h2a)
                    .strafeToLinearHeading(new Vector2d(x2b, y2b), h2b,
                            new TranslationalVelConstraint(pickup1Speed));
            shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, h2b))
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), hShoot);
            

        }

        shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1);
        pickup1 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .strafeToLinearHeading(new Vector2d(x2a, y2a), h2a)
                .strafeToLinearHeading(new Vector2d(x2b, y2b), h2b,
                        new TranslationalVelConstraint(pickup1Speed));
        shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, h2b))
                .strafeToLinearHeading(new Vector2d(xShoot, yShoot), hShoot);
        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            robot.transfer.setPower(1);

            Actions.runBlocking(
                    new ParallelAction(
                            shoot0.build(),
                            manageFlywheel(
                                    shoot0Vel,
                                    shoot0Hood,
                                    flywheel0Time,
                                    x1,
                                    0.501,
                                    shoot0XTolerance,
                                    0.501
                            )

                    )
            );

            Actions.runBlocking(
                    shootAll((int) shoot0Vel, shoot0Time)
            );

            Actions.runBlocking(
                    new ParallelAction(
                            pickup1.build(),
                            manageFlywheel(
                                    shoot1Vel,
                                    shoot1Hood,
                                    pickup1Time,
                                    x2b,
                                    y2b,
                                    pickup1XTolerance,
                                    pickup1YTolerance
                            ),
                            intake(intake1Time),
                            detectObelisk(
                                    obelisk1Time,
                                    x2b,
                                    y2c,
                                    obelisk1XTolerance,
                                    obelisk1YTolerance,
                                    obeliskTurrPos
                            )
                    )
            );

            motif = turret.detectObelisk();

            Actions.runBlocking(
                    new ParallelAction(
                            manageFlywheel(
                                    shoot1Vel,
                                    shoot1Hood,
                                    shoot1Time,
                                    xShoot,
                                    yShoot,
                                    shoot1ToleranceX,
                                    shoot1ToleranceY
                            ),
                            shoot1.build(),
                            prepareShootAll(shoot1Time)
                    )
            );

            Actions.runBlocking(
                    shootAll((int) shoot1Vel, shoot0Time)
            );




            TELE.addData("ID", motif);

            TELE.update();

            robot.intake.setPower(0);

            sleep(5000);
        }

    }
}
