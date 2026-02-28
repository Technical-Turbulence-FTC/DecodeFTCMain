package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Back_Poses.*;
import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueObeliskTurrPos3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.blueTurretShootPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redObeliskTurrPos3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.redTurretShootPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3b;
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
import org.firstinspires.ftc.teamcode.utils.Light;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class Auto_LT_Far extends LinearOpMode {
    public static double shoot0Vel = 3300, shoot0Hood = 0.48;
    double xLeave, yLeave, hLeave;
    public int motif = 0;
    double turretShootPos = 0.0;
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
    double xShoot, yShoot, hShoot;
    double pickupGateXA = 0, pickupGateYA = 0, pickupGateHA = 0;
    double pickupGateXB = 0, pickupGateYB = 0, pickupGateHB = 0;
    double pickupGateXC = 0, pickupGateYC = 0, pickupGateHC = 0;
    public static double flywheel0Time = 1.5;
    boolean gatePickup = true;
    boolean stack3 = true;
    boolean stack2 = true;
    double xStackPickupFarA, yStackPickupFarA, hStackPickupFarA;
    double xStackPickupFarB, yStackPickupFarB, hStackPickupFarB;
    double xStackPickupMiddleA, yStackPickupMiddleA, hStackPickupMiddleA;
    double xStackPickupMiddleB, yStackPickupMiddleB, hStackPickupMiddleB;
    public static int pickupStackSpeed = 17;
    public static int pickupGateSpeed = 25;
    int prevMotif = 0;
    public static double spindexerSpeedIncrease = 0.014;
    public static double shootAllTime = 2;
    // ---- POSITION TOLERANCES ----
    public static double posXTolerance = 5.0;
    public static double posYTolerance = 5.0;
    public static double shootStackTime = 2;
    public static double shootGateTime = 2.5;
    public static double colorSenseTime = 1;
    public static double intakeStackTime = 4.5;
    public static double intakeGateTime = 8;
    double obeliskTurrPos1 = 0.0;
    double obeliskTurrPos2 = 0.0;
    double obeliskTurrPos3 = 0.0;
    public static double endAutoTime = 26;

    // initialize path variables here
    TrajectoryActionBuilder leave3Ball = null;
    TrajectoryActionBuilder leaveFromShoot = null;
    TrajectoryActionBuilder pickup3 = null;
    TrajectoryActionBuilder shoot3 = null;
    TrajectoryActionBuilder pickupGate = null;
    TrajectoryActionBuilder shootGate = null;
    TrajectoryActionBuilder pickup2 = null;
    Pose2d autoStart = new Pose2d(0,0,0);

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

        light = Light.getInstance();

        light.init(robot.light, spindexer, turret);

        robot.limelight.start();

        robot.limelight.pipelineSwitch(1);

        turret = new Turret(robot, TELE, robot.limelight);

        turret.setTurret(turrDefault);

        servos.setSpinPos(spinStartPos);

        servos.setTransferPos(transferServo_out);

        limelightUsed = false;

        while (opModeInInit()) {
            if (limelightUsed){
                Actions.runBlocking(
                        autoActions.detectObelisk(
                                0.1,
                                0.501,
                                0.501,
                                0.501,
                                0.501,
                                turrDefault
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

            if (gamepad2.triangleWasPressed()){
                gatePickup = !gatePickup;
            }
            if (gamepad2.rightBumperWasPressed()){
                stack3 = !stack3;
            }
            if (gamepad2.leftBumperWasPressed()){
                stack2 = !stack2;
            }

            turret.setTurret(turretShootPos);

            robot.hood.setPosition(shoot0Hood);

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;
            }

            if (gamepad2.dpadLeftWasPressed()) {
                turrDefault -= 0.01;
            }

            if (gamepad2.dpadRightWasPressed()) {
                turrDefault += 0.01;
            }

            if (redAlliance) {
                robot.light.setPosition(0.28);

                autoStart = new Pose2d(autoStartRX, autoStartRY, Math.toRadians(autoStartRH));
                drive = new MecanumDrive(hardwareMap, autoStart);
                autoActions = new AutoActions(robot, drive, TELE, servos, flywheel, spindexer, targeting, targetingSettings, turret, light);

                xLeave = rLeaveX;
                yLeave = rLeaveY;
                hLeave = rLeaveH;

                xShoot = rShootX;
                yShoot = rShootY;
                hShoot = rShootH;

                xStackPickupFarA = rStackPickupAX;
                yStackPickupFarA = rStackPickupAY;
                hStackPickupFarA = rStackPickupAH;

                xStackPickupFarB = rStackPickupBX;
                yStackPickupFarB = rStackPickupBY;
                hStackPickupFarB = rStackPickupBH;

                xStackPickupMiddleA = rStackPickupAX;
                yStackPickupMiddleA = rStackPickupAY;
                hStackPickupMiddleA = rStackPickupAH;

                xStackPickupMiddleB = rStackPickupBX;
                yStackPickupMiddleB = rStackPickupBY;
                hStackPickupMiddleB = rStackPickupBH;

                pickupGateXA = rPickupGateXA;
                pickupGateYA = rPickupGateYA;
                pickupGateHA = rPickupGateHA;

                pickupGateXB = rPickupGateXB;
                pickupGateYB = rPickupGateYB;
                pickupGateHB = rPickupGateHB;

                pickupGateXC = rPickupGateXC;
                pickupGateYC = rPickupGateYC;
                pickupGateHC = rPickupGateHC;

                obeliskTurrPos1 = turrDefault + redObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + redObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + redObeliskTurrPos3;
                turretShootPos = turrDefault + redTurretShootPos;

                if (gamepad2.squareWasPressed()){
                    turret.pipelineSwitch(1);
                    robot.limelight.start();
                    limelightUsed = true;
                    gamepad2.rumble(500);
                }
            } else {
                robot.light.setPosition(0.6);

                autoStart = new Pose2d(autoStartBX, autoStartBY, Math.toRadians(autoStartBH));
                drive = new MecanumDrive(hardwareMap, autoStart);
                autoActions = new AutoActions(robot, drive, TELE, servos, flywheel, spindexer, targeting, targetingSettings, turret, light);

                xLeave = bLeaveX;
                yLeave = bLeaveY;
                hLeave = bLeaveH;

                xShoot = bShootX;
                yShoot = bShootY;
                hShoot = bShootH;

                xStackPickupFarA = bStackPickupAX;
                yStackPickupFarA = bStackPickupAY;
                hStackPickupFarA = bStackPickupAH;

                xStackPickupFarB = bStackPickupBX;
                yStackPickupFarB = bStackPickupBY;
                hStackPickupFarB = bStackPickupBH;

                xStackPickupMiddleA = bStackPickupAX;
                yStackPickupMiddleA = bStackPickupAY;
                hStackPickupMiddleA = bStackPickupAH;

                xStackPickupMiddleB = bStackPickupBX;
                yStackPickupMiddleB = bStackPickupBY;
                hStackPickupMiddleB = bStackPickupBH;

                pickupGateXA = bPickupGateXA;
                pickupGateYA = bPickupGateYA;
                pickupGateHA = bPickupGateHA;

                pickupGateXB = bPickupGateXB;
                pickupGateYB = bPickupGateYB;
                pickupGateHB = bPickupGateHB;

                pickupGateXC = bPickupGateXC;
                pickupGateYC = bPickupGateYC;
                pickupGateHC = bPickupGateHC;

                obeliskTurrPos1 = turrDefault + blueObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + blueObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + blueObeliskTurrPos3;
                turretShootPos = turrDefault + blueTurretShootPos;

                if (gamepad2.squareWasPressed()){
                    turret.pipelineSwitch(5);
                    robot.limelight.start();
                    limelightUsed = true;
                    gamepad2.rumble(500);
                }
            }

            leave3Ball = drive.actionBuilder(autoStart)
                    .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));

            leaveFromShoot = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));

            pickup3 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(xStackPickupFarA, yStackPickupFarA), Math.toRadians(hStackPickupFarA))
                    .strafeToLinearHeading(new Vector2d(xStackPickupFarB, yStackPickupFarB), Math.toRadians(hStackPickupFarB),
                            new TranslationalVelConstraint(pickupStackSpeed));

            pickup2 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(xStackPickupMiddleA, yStackPickupMiddleA), Math.toRadians(hStackPickupMiddleA))
                    .strafeToLinearHeading(new Vector2d(xStackPickupMiddleB, yStackPickupMiddleB), Math.toRadians(hStackPickupMiddleB),
                            new TranslationalVelConstraint(pickupStackSpeed));

            shoot3 = drive.actionBuilder(new Pose2d(xStackPickupFarB, yStackPickupFarB, Math.toRadians(hStackPickupFarB)))
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));

            pickupGate = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(pickupGateXA, pickupGateYA), Math.toRadians(pickupGateHA))
                    .waitSeconds(0.2)
                    .strafeToLinearHeading(new Vector2d(pickupGateXB, pickupGateYB), Math.toRadians(pickupGateHB))
                    .strafeToLinearHeading(new Vector2d(pickupGateXC, pickupGateYC), Math.toRadians(pickupGateHC),
                            new TranslationalVelConstraint(pickupGateSpeed));

            shootGate = drive.actionBuilder(new Pose2d(pickupGateXC, pickupGateYC, Math.toRadians(pickupGateHC)))
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));

            TELE.addData("Red?", redAlliance);
            TELE.addData("Turret Default", turrDefault);
            TELE.addData("Gate Cycle?", gatePickup);
            TELE.addData("Pickup Stack Far?", stack3);
            TELE.addData("Pickup Stack Middle?", stack2);
            TELE.addData("Start Position", autoStart);
            TELE.addData("Current Position", drive.localizer.getPose()); // use this to test standstill drift
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        // Currently only shoots; keep this start and modify times and then add extra paths
        if (opModeIsActive()) {
            double stamp = getRuntime();

            robot.transfer.setPower(1);

            startAuto();
            shoot();

            if (stack3){
                cycleStackFar();
                shoot();
            }

            if (stack2){
                cycleStackMiddle();
                shoot();
            }

            while (gatePickup && getRuntime() - stamp < endAutoTime){
                cycleGatePickupBalls();
                if (getRuntime() - stamp > endAutoTime){
                    break;
                }
                cycleGatePrepareShoot();
                if (getRuntime() - stamp > endAutoTime + shootAllTime + 1){
                    break;
                }
                shoot();
            }

            if (gatePickup || stack3){
                leave();
            } else {
                leave3Ball();
            }

            // Actual way to end autonomous in to find final position
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
                        autoActions.shootAllAuto(shootAllTime, spindexerSpeedIncrease, 0.501, 0.501, 0.501)
                )

        );
    }

    void startAuto(){
        Actions.runBlocking(
                new ParallelAction(
                        autoActions.manageShooterAuto(
                                flywheel0Time,
                                0.501,
                                0.501,
                                0.501,
                                true
                        )

                )
        );
    }

    void leave3Ball(){
        assert leave3Ball != null;
        Actions.runBlocking(leave3Ball.build());
    }

    void leave(){
        assert leaveFromShoot != null;
        Actions.runBlocking(leaveFromShoot.build());
    }

    void cycleStackFar(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup3.build(),
                        autoActions.intake(
                                intakeStackTime,
                                xStackPickupFarB,
                                yStackPickupFarB,
                                hStackPickupFarB
                        ),
                        autoActions.detectObelisk(
                                intakeStackTime,
                                xStackPickupFarB,
                                yStackPickupFarB,
                                posXTolerance,
                                posYTolerance,
                                obeliskTurrPos3
                        )

                )
        );

        motif = turret.getObeliskID();

        if (motif == 0) motif = 22;
        prevMotif = motif;

        Actions.runBlocking(
                new ParallelAction(
                        shoot3.build(),
                        autoActions.prepareShootAll(
                                colorSenseTime,
                                shootStackTime,
                                motif,
                                xShoot,
                                yShoot,
                                hShoot)
                )
        );
    }


    void cycleStackMiddle(){
        Actions.runBlocking(
                new ParallelAction(
                        pickup2.build(),
                        autoActions.intake(
                                intakeStackTime,
                                xStackPickupMiddleB,
                                yStackPickupMiddleB,
                                hStackPickupMiddleB
                        ),
                        autoActions.detectObelisk(
                                intakeStackTime,
                                xStackPickupMiddleB,
                                yStackPickupMiddleB,
                                posXTolerance,
                                posYTolerance,
                                obeliskTurrPos2
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
                                shootStackTime,
                                motif,
                                xShoot,
                                yShoot,
                                hShoot)
                )
        );
    }

    void cycleGatePickupBalls(){
        Actions.runBlocking(
                new ParallelAction(
                        pickupGate.build(),
                        autoActions.intake(
                                intakeStackTime,
                                pickupGateXC,
                                pickupGateYC,
                                pickupGateHC
                        ),
                        autoActions.detectObelisk(
                                intakeGateTime,
                                pickupGateXC,
                                pickupGateYC,
                                posXTolerance,
                                posYTolerance,
                                obeliskTurrPos3
                        )
                )
        );

        motif = turret.getObeliskID();

        if (motif == 0) motif = prevMotif;
        prevMotif = motif;
    }

    void cycleGatePrepareShoot(){
        Actions.runBlocking(
                new ParallelAction(
                        shootGate.build(),
                        autoActions.prepareShootAll(
                                colorSenseTime,
                                shootGateTime,
                                motif,
                                xShoot,
                                yShoot,
                                hShoot
                        )
                )
        );
    }
}