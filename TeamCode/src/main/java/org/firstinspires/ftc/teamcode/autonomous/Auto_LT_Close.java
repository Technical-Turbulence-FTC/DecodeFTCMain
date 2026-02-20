package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Turret.limelightUsed;
import static org.firstinspires.ftc.teamcode.utils.Turret.turrDefault;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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
    public static double spindexerSpeedIncrease = 0.02;

    public static double shootAllTime = 4;
    public static double intake1Time = 3.3;
    public static double intake2Time = 3.8;

    public static double intake3Time = 4.2;

    public static double flywheel0Time = 1.5;
    public static double pickup1Speed = 12;
    // ---- POSITION TOLERANCES ----
    public static double posXTolerance = 5.0;
    public static double posYTolerance = 5.0;
    // ---- OBELISK DETECTION ----
    public static double shoot1Time = 2.5;
    public static double shoot2Time = 2.5;
    public static double shoot3Time = 2.5;
    public static double colorSenseTime = 1.2;
    public int motif = 0;
    public static double waitToShoot0 = 0.5;
    public static double waitToPickupGate2 = 0.3;
    public static double pickupStackGateSpeed = 50;
    public static double intake2TimeGate = 3;
    public static double shoot2GateTime = 3;
    public static double endGateTime = 25;
    public static double waitToPickupGateWithPartner = 2;
    public static double waitToPickupGateSolo = 1;
    public static double intakeGateTime = 5;
    public static double shootGateTime = 3;

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
    boolean withPartner = false;
    double obeliskTurrPos1 = 0.0;
    double obeliskTurrPos2 = 0.0;
    double obeliskTurrPos3 = 0.0;
    double waitToPickupGate = 0;

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

        servos.setSpinPos(spinStartPos);

        servos.setTransferPos(transferServo_out);

        limelightUsed = false;

        robot.light.setPosition(1);

        while (opModeInInit()) {

            if (gateCycle){
                servos.setHoodPos(hoodGate0Start);
            } else {
                servos.setHoodPos(shoot0Hood);
            }

            turret.setTurret(turrDefault);

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;
            }

            if (gamepad2.dpadLeftWasPressed()) {
                turrDefault -= 0.01;
            }

            if (gamepad2.dpadRightWasPressed()) {
                turrDefault += 0.01;
            }

            if (gamepad2.rightBumperWasPressed()){
                ballCycles++;
            }
            if (gamepad2.leftBumperWasPressed()){
                ballCycles--;
            }

            if (gamepad2.squareWasPressed()){
                turret.pipelineSwitch(1);
                robot.limelight.start();
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

                obeliskTurrPos1 = turrDefault + blueObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + blueObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + blueObeliskTurrPos3;
            }

            shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(h1));

            if (gateCycle){
                pickup1 = drive.actionBuilder(new Pose2d(xShootGate, yShootGate, Math.toRadians(hShootGate)))
                        .strafeToLinearHeading(new Vector2d(x2a, y2a), Math.toRadians(h2a))
                        .strafeToLinearHeading(new Vector2d(x2b, y2b), Math.toRadians(h2b),
                                new TranslationalVelConstraint(pickupStackGateSpeed));
            } else {
                pickup1 = drive.actionBuilder(new Pose2d(x1, y1, Math.toRadians(h1)))
                        .strafeToLinearHeading(new Vector2d(x2a, y2a), Math.toRadians(h2a))
                        .strafeToLinearHeading(new Vector2d(x2b, y2b), Math.toRadians(h2b),
                                new TranslationalVelConstraint(pickup1Speed));
            }

            if (gateCycle){
                shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, Math.toRadians(h2b)))
                        .strafeToLinearHeading(new Vector2d(xLeaveGate, yLeaveGate), Math.toRadians(hLeaveGate));
            } else if (ballCycles < 2){
                shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, Math.toRadians(h2b)))
                        .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));
            } else {
                shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, Math.toRadians(h2b)))
                        .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));
            }

            pickup2 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(x3a, y3a), Math.toRadians(h3a))
                    .strafeToLinearHeading(new Vector2d(x3b, y3b), Math.toRadians(h3b),
                            new TranslationalVelConstraint(pickup1Speed));

            if (gateCycle){
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xShootGate, yShootGate), Math.toRadians(hShootGate));
            } else if (ballCycles < 3){
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));
            } else {
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));
            }

            pickup3 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(x4a, y4a), Math.toRadians(h4a))
                    .strafeToLinearHeading(new Vector2d(x4b, y4b), Math.toRadians(h4b),
                            new TranslationalVelConstraint(pickup1Speed));

            shoot3 = drive.actionBuilder(new Pose2d(x4b, y4b, Math.toRadians(h4b)))
                    .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));

            shoot0ToPickup2 = drive.actionBuilder(new Pose2d(0,0,0))
                    .strafeToLinearHeading(new Vector2d(xShoot0, yShoot0), Math.toRadians(hShoot0))
                    .waitSeconds(waitToPickupGate2)
                    .strafeToLinearHeading(new Vector2d(x3a, y3a), Math.toRadians(h3a))
                    .strafeToLinearHeading(new Vector2d(x3b, y3b), Math.toRadians(h3b),
                            new TranslationalVelConstraint(pickupStackGateSpeed));

            if (withPartner){
                waitToPickupGate = waitToPickupGateWithPartner;
            } else {
                waitToPickupGate = waitToPickupGateSolo;
            }

            gateCyclePickup = drive.actionBuilder(new Pose2d(xShootGate, yShootGate, Math.toRadians(hShootGate)))
                    .strafeToLinearHeading(new Vector2d(pickupGateAX, pickupGateAY), Math.toRadians(pickupGateAH))
                    .waitSeconds(waitToPickupGate)
                    .strafeToLinearHeading(new Vector2d(pickupGateBX, pickupGateBY), Math.toRadians(pickupGateBH));

            gateCycleShoot = drive.actionBuilder(new Pose2d(pickupGateBX, pickupGateBY, Math.toRadians(pickupGateBH)))
                    .strafeToLinearHeading(new Vector2d(xShootGate, yShootGate), Math.toRadians(hShootGate));

            TELE.addData("Red?", redAlliance);
            TELE.addData("Turret Default", turrDefault);
            TELE.addData("Ball Cycles", ballCycles);

            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            double stamp = getRuntime();

            robot.transfer.setPower(1);

            if (gateCycle){
                shoot0Gate();
                cycleStackMiddleGate();

                while (getRuntime() - stamp < endGateTime){
                    cycleGateIntake();
                    cycleGateShoot();
                }

            } else {
                startAuto();
                shoot();

                if (ballCycles > 0){
                    cycleStackClose();
                    shoot();
                }

                if (ballCycles > 1){
                    cycleStackMiddle();
                    shoot();
                }

                if (ballCycles > 2){
                    cycleStackFar();
                    shoot();
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

    void shoot(){
        Actions.runBlocking(
                new ParallelAction(
                        autoActions.shootAllAuto(shootAllTime, spindexerSpeedIncrease)
                )

        );
    }

    void startAuto() {
        assert shoot0 != null;

        Actions.runBlocking(
                new ParallelAction(
                        shoot0.build(),
                        autoActions.manageShooterAuto(
                                flywheel0Time,
                                x1,
                                y1,
                                h1
                        )
                )
        );
    }

    void cycleStackClose(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup1.build(),
                        autoActions.intake(
                                intake1Time,
                                x2b,
                                y2b,
                                h2b
                        ),
                        autoActions.detectObelisk(
                                intake1Time,
                                x2b,
                                y2b,
                                posXTolerance,
                                posYTolerance,
                                obeliskTurrPos1
                        )

                )
        );

        motif = turret.getObeliskID();

        if (motif == 0) motif = 22;
        prevMotif = motif;

        double posX;
        double posY;
        double posH;
        if (ballCycles > 1){
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

    void cycleStackMiddle(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup2.build(),
                        autoActions.intake(
                                intake2Time,
                                x3b,
                                y3b,
                                h3b
                        ),
                        autoActions.detectObelisk(
                                intake2Time,
                                x3b,
                                y3b,
                                posXTolerance,
                                posYTolerance,
                                obeliskTurrPos2
                        )

                )
        );

        motif = turret.getObeliskID();

        if (motif == 0) motif = prevMotif;
        prevMotif = motif;

        double posX;
        double posY;
        double posH;
        if (ballCycles > 2){
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

    void cycleStackFar(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup3.build(),
                        autoActions.intake(
                                intake3Time,
                                x4b,
                                y4b,
                                h4b
                        ),
                        autoActions.detectObelisk(
                                intake3Time,
                                x4b,
                                y4b,
                                posXTolerance,
                                posYTolerance,
                                obeliskTurrPos3
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

    void shoot0Gate(){
        Actions.runBlocking(
                new ParallelAction(
                        shoot0ToPickup2.build(),
                        new SequentialAction(
                                new ParallelAction(
                                        autoActions.manageShooterManual(
                                                waitToShoot0,
                                                0.501,
                                                velGate0Start,
                                                hoodGate0Start,
                                                velGate0Start,
                                                hoodGate0Start,
                                                0.501
                                        )
                                ),
                                autoActions.shootAllManual(
                                        shootAllTime,
                                        hood0MoveTime,
                                        spindexerSpeedIncrease,
                                        velGate0Start,
                                        hoodGate0Start,
                                        velGate0End,
                                        hoodGate0End,
                                        0.501),
                                autoActions.intake(
                                        intake2TimeGate,
                                        xShootGate,
                                        yShootGate,
                                        hShootGate
                                )
                        )
                )
        );
    }

    void cycleStackMiddleGate(){
        servos.setSpinPos(spinStartPos);
        Actions.runBlocking(
                new ParallelAction(
                        shoot2.build(),
                        new SequentialAction(
                                new ParallelAction(
                                        autoActions.manageShooterAuto(
                                                shoot2GateTime,
                                                xShootGate,
                                                yShootGate,
                                                hShootGate
                                        ),
                                        autoActions.Wait(shoot2GateTime)
                                ),
                                autoActions.shootAllAuto(shootAllTime, spindexerSpeedIncrease)
                        )
                )
        );
    }

    void cycleGateIntake(){
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

    void cycleGateShoot(){
        servos.setSpinPos(spinStartPos);
        Actions.runBlocking(
                new ParallelAction(
                        gateCycleShoot.build(),
                        new SequentialAction(
                                new ParallelAction(
                                        autoActions.manageShooterAuto(
                                                shootGateTime,
                                                xShootGate,
                                                yShootGate,
                                                hShootGate
                                        ),
                                        autoActions.Wait(shootGateTime)
                                ),
                                autoActions.shootAllAuto(shootAllTime, spindexerSpeedIncrease)
                        )
                )
        );
    }
}