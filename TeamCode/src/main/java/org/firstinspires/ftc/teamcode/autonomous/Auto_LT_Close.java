package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos0;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3b;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Turret.limelightUsed;
import static org.firstinspires.ftc.teamcode.utils.Turret.turrDefault;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.actions.AutoActions;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Light;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class Auto_LT_Close extends LinearOpMode {
    public static double shoot0Vel = 2300, shoot0Hood = 0.93;
    public static double velGate0Start = 2700, hoodGate0Start = 0.6;

    public static double velGate0End = 2700, hoodGate0End = 0.35;
    public static double hood0MoveTime = 2;
    public static double spindexerSpeedIncrease = 0.014;

    public static double shootAllTime  = 6.0;
    public static double intake1Time = 3.3;
    public static double intake2Time = 4.2;

    public static double intake3Time = 5.4;

    public static double flywheel0Time = 1.9;
    public static double pickup1Speed = 14;
    // ---- POSITION TOLERANCES ----
    public static double posXTolerance = 5.0;
    public static double posYTolerance = 5.0;
    // ---- OBELISK DETECTION ----
    public static double shoot1Time = 2.5;
    public static double shoot2Time = 2.5;
    public static double shoot3Time = 2.5;
    public static double colorSenseTime = 1.2;
    public static double waitToShoot0 = 0.5;
    public static double waitToPickupGate2 = 0.3;
    public static double pickupStackGateSpeed = 19;
    public static double intake2TimeGate = 5;
    public static double shoot2GateTime = 1.7;
    public static double endGateTime = 22;
    public static double waitToPickupGateWithPartner = 0.7;
    public static double waitToPickupGateSolo = 0.01;
    public static double intakeGateTime = 5.6;
    public static double shootGateTime = 1.5;
    public static double shoot1GateTime = 1.7;
    public static double intake1GateTime = 3.3;
    public static double lastShootTime = 27;

    public static double openGateX = 26;
    public static double openGateY = 48;
    public  static double openGateH = Math.toRadians(155);

    Robot robot;
    MultipleTelemetry TELE;
    MecanumDrive drive;
    Servos servos;
    Spindexer spindexer;
    Flywheel flywheel;
    Turret turret;
    Targeting targeting;
    Targeting.Settings targetingSettings;
    AutoActions autoActions;
    Light light;

    int motif = 0;
    double x1, y1, h1;
    double x2a, y2a, h2a, t2a;

    double x2b, y2b, h2b, t2b;
    double x2c, y2c, h2c, t2c;

    double x3a, y3a, h3a;
    double x3b, y3b, h3b;
    double x4a, y4a, h4a;
    double x4b, y4b, h4b;

    double xShoot, yShoot, hShoot;
    double xShoot0, yShoot0, hShoot0;
    double pickupGateAX, pickupGateAY, pickupGateAH;
    double pickupGateBX, pickupGateBY, pickupGateBH;
    double xShootGate, yShootGate, hShootGate;
    double xLeave, yLeave, hLeave;
    double xLeaveGate, yLeaveGate, hLeaveGate;

    int ballCycles = 3;
    int prevMotif = 0;
    boolean gateCycle = true;
    boolean withPartner = true;
    double obeliskTurrPos1 = 0.0;
    double obeliskTurrPos2 = 0.0;
    double obeliskTurrPos3 = 0.0;
    double waitToPickupGate = 0;
    double obeliskTurrPosAutoStart = 0;

    // initialize path variables here
    TrajectoryActionBuilder shoot0 = null;
    TrajectoryActionBuilder pickup1 = null;
    TrajectoryActionBuilder shoot1 = null;
    TrajectoryActionBuilder pickup2 = null;
    TrajectoryActionBuilder shoot2 = null;
    TrajectoryActionBuilder pickup3 = null;
    TrajectoryActionBuilder shoot3 = null;
    TrajectoryActionBuilder shoot0ToPickup2 = null;
    TrajectoryActionBuilder gateCyclePickup = null;
    TrajectoryActionBuilder gateCycleShoot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        flywheel = new Flywheel(hardwareMap);

        targeting = new Targeting();
        targetingSettings = new Targeting.Settings(0.0, 0.0);

        spindexer = new Spindexer(hardwareMap);

        servos = new Servos(hardwareMap);

        turret = new Turret(robot, TELE, robot.limelight);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        light = Light.getInstance();

        light.init(robot.light, spindexer, turret);

        autoActions = new AutoActions(robot, drive, TELE, servos, flywheel, spindexer, targeting, targetingSettings, turret, light);

        servos.setSpinPos(spindexer_intakePos1);

        servos.setTransferPos(transferServo_out);
        limelightUsed = false;

        robot.light.setPosition(1);

        hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint").resetPosAndIMU();

        while (opModeInInit()) {
            if (limelightUsed && !gateCycle){
                Actions.runBlocking(
                        autoActions.detectObelisk(
                                0.1,
                                0.501,
                                0.501,
                                0.501,
                                0.501,
                                obeliskTurrPosAutoStart
                        )
                );
                motif = turret.getObeliskID();

                if (motif == 21){
                    AutoActions.firstSpindexShootPos = spindexer_outtakeBall1;
                } else if (motif == 22){
                    AutoActions.firstSpindexShootPos = spindexer_outtakeBall3b;
                } else {
                    AutoActions.firstSpindexShootPos = spindexer_outtakeBall2;
                }
            }

            if (!gateCycle) {
                turret.pipelineSwitch(1);
            } else if (redAlliance) {
                turret.pipelineSwitch(4);
            } else {
                turret.pipelineSwitch(2);
            }

            if (gateCycle) {
                servos.setHoodPos(hoodGate0Start);
            } else {
                servos.setHoodPos(shoot0Hood);
            }

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;
            }

            if (gamepad2.dpadLeftWasPressed()) {
                turrDefault -= 0.01;
            }

            if (gamepad2.dpadRightWasPressed()) {
                turrDefault += 0.01;
            }

            if (gamepad2.rightBumperWasPressed()) {
                ballCycles++;
            }
            if (gamepad2.leftBumperWasPressed()) {
                ballCycles--;
            }

            if (gamepad2.triangleWasPressed()){
                gateCycle = !gateCycle;
            }



            if (gamepad2.squareWasPressed()) {

                drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
                robot.limelight.start();
                limelightUsed = true;

                gamepad2.rumble(500);
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
                x2b = rx2b;
                y2b = ry2b;
                h2b = rh2b;
                x3a = rx3a;
                y3a = ry3a;
                h3a = rh3a;
                x3b = rx3b;
                y3b = ry3b;
                h3b = rh3b;
                x4a = rx4a;
                y4a = ry4a;
                h4a = rh4a;
                x4b = rx4b;
                y4b = ry4b;
                h4b = rh4b;

                xShoot = rShootX;
                yShoot = rShootY;
                hShoot = rShootH;
                xLeave = rLeaveX;
                yLeave = rLeaveY;
                hLeave = rLeaveH;

                xShoot0 = rShoot0X;
                yShoot0 = rShoot0Y;
                hShoot0 = rShoot0H;
                xShootGate = rShootGateX;
                yShootGate = rShootGateY;
                hShootGate = rShootGateH;
                xLeaveGate = rLeaveGateX;
                yLeaveGate = rLeaveGateY;
                hLeaveGate = rLeaveGateH;

                pickupGateAX = rPickupGateAX;
                pickupGateAY = rPickupGateAY;
                pickupGateAH = rPickupGateAH;
                pickupGateBX = rPickupGateBX;
                pickupGateBY = rPickupGateBY;
                pickupGateBH = rPickupGateBH;

                obeliskTurrPosAutoStart = turrDefault + redObeliskTurrPos0;
                obeliskTurrPos1 = turrDefault + redObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + redObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + redObeliskTurrPos3;
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
                x2b = bx2b;
                y2b = by2b;
                h2b = bh2b;
                x3a = bx3a;
                y3a = by3a;
                h3a = bh3a;
                x3b = bx3b;
                y3b = by3b;
                h3b = bh3b;
                x4a = bx4a;
                y4a = by4a;
                h4a = bh4a;
                x4b = bx4b;
                y4b = by4b;
                h4b = bh4b;

                xShoot = bShootX;
                yShoot = bShootY;
                hShoot = bShootH;
                xLeave = bLeaveX;
                yLeave = bLeaveY;
                hLeave = bLeaveH;

                xShoot0 = bShoot0X;
                yShoot0 = bShoot0Y;
                hShoot0 = bShoot0H;
                xShootGate = bShootGateX;
                yShootGate = bShootGateY;
                hShootGate = bShootGateH;
                xLeaveGate = bLeaveGateX;
                yLeaveGate = bLeaveGateY;
                hLeaveGate = bLeaveGateH;

                pickupGateAX = bPickupGateAX;
                pickupGateAY = bPickupGateAY;
                pickupGateAH = bPickupGateAH;
                pickupGateBX = bPickupGateBX;
                pickupGateBY = bPickupGateBY;
                pickupGateBH = bPickupGateBH;

                obeliskTurrPosAutoStart = turrDefault + redObeliskTurrPos0;
                obeliskTurrPos1 = turrDefault + blueObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + blueObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + blueObeliskTurrPos3;
            }

            if (gateCycle) {
                shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(xShoot0, yShoot0), Math.toRadians(hShoot0));
            } else {
                shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(h1));
            }

            if (gateCycle) {
                pickup2 = shoot0.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(x3a, y3a), Math.toRadians(h3a))
                        .strafeToLinearHeading(new Vector2d(x3b, y3b), Math.toRadians(h3b),
                                new TranslationalVelConstraint(pickupStackGateSpeed));
            } else {
                pickup2 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                        .strafeToLinearHeading(new Vector2d(x3a, y3a), Math.toRadians(h3a))
                        .strafeToLinearHeading(new Vector2d(x3b, y3b), Math.toRadians(h3b),
                                new TranslationalVelConstraint(pickup1Speed));
            }

            if (gateCycle&& withPartner) {
                shoot2 = pickup2.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(openGateX, openGateY), Math.toRadians(openGateH))
                        .strafeToLinearHeading(new Vector2d(xShootGate, yShootGate), Math.toRadians(pickupGateAH));
            } else if (gateCycle) {
                shoot2 = pickup2.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(xShootGate, yShootGate), Math.toRadians(hShootGate));
            } else if (ballCycles < 3) {
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));
            } else {
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));
            }

            gateCyclePickup = shoot2.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(pickupGateAX, pickupGateAY), Math.toRadians(pickupGateAH))
                    .waitSeconds(waitToPickupGate)
                    .strafeToLinearHeading(new Vector2d(pickupGateBX, pickupGateBY), Math.toRadians(pickupGateBH))
                    .waitSeconds(0.1)
                    .strafeToLinearHeading(new Vector2d(pickupGateCX, pickupGateCY), Math.toRadians(pickupGateCH),
                            new TranslationalVelConstraint(13));

            gateCycleShoot = gateCyclePickup.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(xShootGate, yShootGate), Math.toRadians(hShootGate));


            if (gateCycle) {
                pickup1 = gateCycleShoot.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(x2a, y2a), Math.toRadians(h2a))
                        .strafeToLinearHeading(new Vector2d(x2b, y2b), Math.toRadians(h2b),
                                new TranslationalVelConstraint(pickupStackGateSpeed));
            } else {
                pickup1 = drive.actionBuilder(new Pose2d(x1, y1, Math.toRadians(h1)))
                        .strafeToLinearHeading(new Vector2d(x2a, y2a), Math.toRadians(h2a))
                        .strafeToLinearHeading(new Vector2d(x2b, y2b), Math.toRadians(h2b),
                                new TranslationalVelConstraint(pickup1Speed));
            }


            if (gateCycle) {
                shoot1 = pickup1.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(xLeaveGate, yLeaveGate), Math.toRadians(hLeaveGate));
            } else if (ballCycles < 2) {
                shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, Math.toRadians(h2b)))
                        .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));
            } else {
                shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, Math.toRadians(h2b)))
                        .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));
            }

            pickup3 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(x4a, y4a), Math.toRadians(h4a))
                    .strafeToLinearHeading(new Vector2d(x4b, y4b), Math.toRadians(h4b),
                            new TranslationalVelConstraint(pickup1Speed));

            shoot3 = drive.actionBuilder(new Pose2d(x4b, y4b, Math.toRadians(h4b)))
                    .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));

            if (withPartner) {
                waitToPickupGate = waitToPickupGateWithPartner;
            } else {
                waitToPickupGate = waitToPickupGateSolo;
            }

            teleStart = drive.localizer.getPose();
            TELE.addData("Red?", redAlliance);
            TELE.addData("Turret Default", turrDefault);
            TELE.addData("Gate Cycle?", gateCycle);
            TELE.addData("Ball Cycles", ballCycles);
            TELE.addData("Limelight Started?", limelightUsed);
            TELE.addData("Motif", motif);

            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            double stamp = getRuntime();

            robot.transfer.setPower(1);


            if (gateCycle) {
                startAutoGate();
                shoot(0.501, 0.501, 0.501);
                cycleStackMiddleGate();
                shoot(0.501,0.501, 0.501);

                while (getRuntime() - stamp < endGateTime) {
                    cycleGateIntake();
                    if (getRuntime() - stamp < lastShootTime) {
                        cycleGateShoot();
                        shoot(0.501, 0.501, 0.501);
                    }
                }
                cycleStackCloseIntakeGate();

                if (getRuntime() - stamp < lastShootTime) {
                    cycleStackCloseShootGate();
                }

                shoot(0.501, 0.501, 0.501);

            } else {



                startAuto();
                shoot(0.501, 0.501,0.501);

                if (ballCycles > 0) {
                    cycleStackClose();
                    shoot(xShoot, yShoot, hShoot);
                }

                if (ballCycles > 1) {
                    cycleStackMiddle();
                    shoot(xShoot, yShoot, hShoot);
                }

                if (ballCycles > 2) {
                    cycleStackFar();
                    shoot(xLeave, yLeave, hLeave);
                }
            }

            while (opModeIsActive()) {

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                flywheel.manageFlywheel(0);
                robot.transfer.setPower(0);

                TELE.addLine("finished");
                TELE.update();
            }

        }

    }

    void shoot(double x, double y, double z) {
        Actions.runBlocking(autoActions.shootAllAuto(shootAllTime, spindexerSpeedIncrease, x, y, z));
    }

    void startAuto() {
        assert shoot0 != null;

        Actions.runBlocking(
                new ParallelAction(
                        shoot0.build(),
                        autoActions.prepareShootAll(
                                0.8,
                                flywheel0Time,
                                motif,
                                x1,
                                y1,
                                h1
                        )
                )
        );

    }



    void startAutoGate() {
        assert shoot0 != null;

        Actions.runBlocking(
                new ParallelAction(
                        shoot0.build(),
                        autoActions.prepareShootAll(
                                colorSenseTime,
                                flywheel0Time,
                                motif,
                                xShoot0,
                                yShoot0,
                                hShoot0
                        )
                )
        );


    }

    void cycleStackClose() {
        Actions.runBlocking(
                new ParallelAction(
                        pickup1.build(),
                        autoActions.intake(
                                intake1Time,
                                x2b,
                                y2b,
                                h2b
                        )
                )
        );

        double posX;
        double posY;
        double posH;
        if (ballCycles > 1) {
            posX = xShoot;
            posY = yShoot;
            posH = hShoot;
        } else {
            posX = xLeave;
            posY = yLeave;
            posH = hLeave;
        }

        Actions.runBlocking(
                new ParallelAction(
                        shoot1.build(),
                        autoActions.prepareShootAll(
                                colorSenseTime,
                                shoot1Time,
                                motif,
                                posX,
                                posY,
                                posH
                        )
                )
        );
    }

    void cycleStackMiddle() {
        Actions.runBlocking(
                new ParallelAction(
                        pickup2.build(),
                        autoActions.intake(
                                intake2Time,
                                x3b,
                                y3b,
                                h3b
                        )
                )
        );

        double posX;
        double posY;
        double posH;
        if (ballCycles > 2) {
            posX = xShoot;
            posY = yShoot;
            posH = hShoot;
        } else {
            posX = xLeave;
            posY = yLeave;
            posH = hLeave;
        }

        Actions.runBlocking(
                new ParallelAction(
                        shoot2.build(),
                        autoActions.prepareShootAll(
                                colorSenseTime,
                                shoot2Time,
                                motif,
                                posX,
                                posY,
                                posH)
                )
        );
    }

    void cycleStackFar() {
        Actions.runBlocking(
                new ParallelAction(
                        pickup3.build(),
                        autoActions.intake(
                                intake3Time,
                                x4b,
                                y4b,
                                h4b
                        )
                )
        );

        motif = turret.getObeliskID();

        if (motif == 0) motif = prevMotif;
        prevMotif = motif;

        Actions.runBlocking(
                new ParallelAction(
                        shoot3.build(),
                        autoActions.prepareShootAll(
                                colorSenseTime,
                                shoot3Time,
                                motif,
                                xLeave,
                                yLeave,
                                hLeave
                        )
                )
        );
    }

    void cycleStackMiddleGate() {
        drive.updatePoseEstimate();
        pickup2 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(x3a, y3a), Math.toRadians(h3a))
                .strafeToLinearHeading(new Vector2d(x3b, y3b), Math.toRadians(h3b),
                        new TranslationalVelConstraint(pickupStackGateSpeed));
        Actions.runBlocking(
                new ParallelAction(
                        pickup2.build(),
                        autoActions.intake(
                                intake2TimeGate,
                                x3b,
                                y3b,
                                h3b
                        )
                )
        );

        servos.setSpinPos(spinStartPos);
        Actions.runBlocking(
                new ParallelAction(
                        shoot2.build(),
                        autoActions.prepareShootAll(
                                colorSenseTime,
                                shoot2Time,
                                motif,
                                xShootGate,
                                yShootGate,
                                pickupGateAH)
                )
        );
    }

    void cycleGateIntake() {
        drive.updatePoseEstimate();

        Actions.runBlocking(
                new ParallelAction(
                        gateCyclePickup.build(),
                        autoActions.intake(
                                intakeGateTime,
                                xShootGate,
                                yShootGate,
                                hShootGate
                        )
                )
        );
    }

    void cycleGateShoot() {

        drive.updatePoseEstimate();
        servos.setSpinPos(spinStartPos);

        gateCycleShoot = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(xShootGate, yShootGate), Math.toRadians(pickupGateAH));

        Actions.runBlocking(
                new ParallelAction(
                        gateCycleShoot.build(),
                        autoActions.manageShooterAuto(
                                shootGateTime,
                                xShootGate,
                                yShootGate,
                                pickupGateAH,
                                false
                        )
                )
        );
    }

    void cycleStackCloseIntakeGate() {
        drive.updatePoseEstimate();

        pickup1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(x2a, y2a), Math.toRadians(h2a))
                .strafeToLinearHeading(new Vector2d(x2b, y2b), Math.toRadians(h2b),
                        new TranslationalVelConstraint(pickupStackGateSpeed));


        Actions.runBlocking(
                new ParallelAction(
                        pickup1.build(),
                        autoActions.intake(
                                intake1GateTime,
                                xShootGate,
                                yShootGate,
                                hShootGate
                        )
                )
        );
    }

    void cycleStackCloseShootGate(){
        servos.setSpinPos(spinStartPos);
        drive.updatePoseEstimate();

        shoot1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(xLeaveGate, yLeaveGate), Math.toRadians(hLeaveGate));

        Actions.runBlocking(
                new ParallelAction(
                        shoot1.build(),
                        autoActions.manageShooterAuto(
                                shoot1GateTime,
                                xLeaveGate,
                                yLeaveGate,
                                hLeaveGate,
                                false
                        )
                )
        );
    }
}