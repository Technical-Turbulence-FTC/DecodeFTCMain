package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Turret.limelightUsed;
import static org.firstinspires.ftc.teamcode.utils.Turret.turrDefault;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.actions.AutoActions;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class Auto_LT_Close extends LinearOpMode {
    public static double shoot0Vel = 2300, shoot0Hood = 0.93;
    public static double spindexerSpeedIncrease = 0.005;

    // These values are ADDED to turrDefault
    public static double redObeliskTurrPos1 = 0.12;
    public static double redObeliskTurrPos2 = 0.13;
    public static double redObeliskTurrPos3 = 0.14;
    public static double blueObeliskTurrPos1 = -0.12;
    public static double blueObeliskTurrPos2 = -0.13;
    public static double blueObeliskTurrPos3 = -0.14;
    public static double redTurretShootPos = 0.1;
    public static double blueTurretShootPos = -0.14;

    double obeliskTurrPos1 = 0.0;
    double obeliskTurrPos2 = 0.0;
    double obeliskTurrPos3 = 0.0;

    double turretShootPos = 0.0;

    public static double shootAllTime = 2;
    public static double intake1Time = 3.3;
    public static double intake2Time = 3.8;

    public static double intake3Time = 4.2;

    public static double flywheel0Time = 3.5;
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
    double x1, y1, h1;

    double x2a, y2a, h2a, t2a;

    double x2b, y2b, h2b, t2b;
    double x2c, y2c, h2c, t2c;

    double x3a, y3a, h3a;
    double x3b, y3b, h3b;
    double x4a, y4a, h4a;
    double x4b, y4b, h4b;

    double xShoot, yShoot, hShoot;
    double xGate, yGate, hGate;
    double xPrep, yPrep, hPrep;
    double xLeave, yLeave, hLeave;

    private double shoot1Tangent;

    int ballCycles = 3;
    int prevMotif = 0;


    // initialize path variables here
    TrajectoryActionBuilder shoot0 = null;
    TrajectoryActionBuilder pickup1 = null;
    TrajectoryActionBuilder shoot1 = null;
    TrajectoryActionBuilder pickup2 = null;
    TrajectoryActionBuilder shoot2 = null;
    TrajectoryActionBuilder pickup3 = null;
    TrajectoryActionBuilder shoot3 = null;

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

        autoActions = new AutoActions(robot, drive, TELE, servos, flywheel, spindexer, targeting, targetingSettings, turret);

        servos.setSpinPos(spinStartPos);

        servos.setTransferPos(transferServo_out);

        limelightUsed = false;

        robot.light.setPosition(1);

        while (opModeInInit()) {

            servos.setHoodPos(shoot0Hood);
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
                robot.limelight.start();
                robot.limelight.pipelineSwitch(1);
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
                xPrep = rxPrep;
                yPrep = ryPrep;
                hPrep = rhPrep;
                xShoot = rShootX;
                yShoot = rShootY;
                hShoot = rShootH;
                xLeave = rLeaveX;
                yLeave = rLeaveY;
                hLeave = rLeaveH;

                obeliskTurrPos1 = turrDefault + redObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + redObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + redObeliskTurrPos3;
                turretShootPos = turrDefault + redTurretShootPos;

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

                xPrep = bxPrep;
                yPrep = byPrep;
                hPrep = bhPrep;
                xShoot = bShootX;
                yShoot = bShootY;
                hShoot = bShootH;
                xLeave = bLeaveX;
                yLeave = bLeaveY;
                hLeave = bLeaveH;

                obeliskTurrPos1 = turrDefault + blueObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + blueObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + blueObeliskTurrPos3;
                turretShootPos = turrDefault + blueTurretShootPos;

            }

            shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                    .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(h1));

            pickup1 = drive.actionBuilder(new Pose2d(x1, y1, Math.toRadians(h1)))
                    .strafeToLinearHeading(new Vector2d(x2a, y2a), Math.toRadians(h2a))
                    .strafeToLinearHeading(new Vector2d(x2b, y2b), Math.toRadians(h2b),
                            new TranslationalVelConstraint(pickup1Speed));

            if (ballCycles < 2){
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

            if (ballCycles < 3){
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));
            } else {
                shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, Math.toRadians(h3b)))
                        .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hLeave));
            }

            pickup3 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(x4a, y4a), Math.toRadians(h4a))
                    .strafeToLinearHeading(new Vector2d(x4b, y4b), Math.toRadians(h4b),
                            new TranslationalVelConstraint(pickup1Speed));

            shoot3 = drive.actionBuilder(new Pose2d(x4b, y4b, Math.toRadians(h4b)))
                    .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));

            TELE.addData("Red?", redAlliance);
            TELE.addData("Turret Default", turrDefault);
            TELE.addData("Ball Cycles", ballCycles);

            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            robot.transfer.setPower(1);

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

            while (opModeIsActive()) {

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                flywheel.manageFlywheel(0);

                TELE.addLine("finished");
                TELE.update();
            }

        }

    }

    void shoot(){
        Actions.runBlocking(
                new ParallelAction(
                        autoActions.manageShooterAuto(
                                shootAllTime,
                                0.501,
                                0.501,
                                0.501,
                                0.501,
                                0.501,
                                false
                        ),
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
                                posXTolerance,
                                posYTolerance,
                                h1,
                                false
                        )

                )
        );
    }

    void cycleStackClose(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup1.build(),
                        autoActions.manageShooterAuto(
                                intake1Time,
                                x2b,
                                y2b,
                                posXTolerance,
                                posYTolerance,
                                h2b,
                                true
                        ),
                        autoActions.intake(intake1Time),
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
                        autoActions.manageShooterAuto(
                                shoot1Time,
                                posX,
                                posY,
                                posXTolerance,
                                posYTolerance,
                                posH,
                                false
                        ),
                        shoot1.build(),
                        autoActions.prepareShootAll(colorSenseTime, shoot1Time, motif)
                )
        );
    }

    void cycleStackMiddle(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup2.build(),
                        autoActions.manageShooterAuto(
                                intake2Time,
                                x3b,
                                y3b,
                                posXTolerance,
                                posYTolerance,
                                h3b,
                                true
                        ),
                        autoActions.intake(intake2Time),
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
                        autoActions.manageShooterAuto(
                                shoot2Time,
                                posX,
                                posY,
                                posXTolerance,
                                posYTolerance,
                                posH,
                                false
                        ),
                        shoot2.build(),
                        autoActions.prepareShootAll(colorSenseTime, shoot2Time, motif)
                )
        );
    }

    void cycleStackFar(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup3.build(),
                        autoActions.manageShooterAuto(
                                intake3Time,
                                x4b,
                                y4b,
                                posXTolerance,
                                posYTolerance,
                                h4b,
                                true
                        ),
                        autoActions.intake(intake3Time),
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
                        autoActions.manageShooterAuto(
                                shoot3Time,
                                xLeave,
                                yLeave,
                                posXTolerance,
                                posYTolerance,
                                hLeave,
                                false
                        ),
                        shoot3.build(),
                        autoActions.prepareShootAll(colorSenseTime, shoot3Time, motif)
                )
        );
    }
}