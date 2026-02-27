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
    double pickupGateX = 0, pickupGateY = 0, pickupGateH = 0;
    public static double flywheel0Time = 1.5;
    boolean gatePickup = true;
    boolean stack3 = true;
    double xStackPickupA, yStackPickupA, hStackPickupA;
    double xStackPickupB, yStackPickupB, hStackPickupB;
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

        while (opModeInInit()) {

            if (gamepad2.leftBumperWasPressed()){
                gatePickup = !gatePickup;
            }
            if (gamepad2.rightBumperWasPressed()){
                stack3 = !stack3;
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

                xStackPickupA = rStackPickupAX;
                yStackPickupA = rStackPickupAY;
                hStackPickupA = rStackPickupAH;

                xStackPickupB = rStackPickupBX;
                yStackPickupB = rStackPickupBY;
                hStackPickupB = rStackPickupBH;

                pickupGateX = rPickupGateX;
                pickupGateY = rPickupGateY;
                pickupGateH = rPickupGateH;

                obeliskTurrPos1 = turrDefault + redObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + redObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + redObeliskTurrPos3;
                turretShootPos = turrDefault + redTurretShootPos;

                if (gamepad2.squareWasPressed()){
                    turret.pipelineSwitch(4);
                    robot.limelight.start();
                    drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

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

                xStackPickupA = bStackPickupAX;
                yStackPickupA = bStackPickupAY;
                hStackPickupA = bStackPickupAH;

                xStackPickupB = bStackPickupBX;
                yStackPickupB = bStackPickupBY;
                hStackPickupB = bStackPickupBH;

                pickupGateX = bPickupGateX;
                pickupGateY = bPickupGateY;
                pickupGateH = bPickupGateH;

                obeliskTurrPos1 = turrDefault + blueObeliskTurrPos1;
                obeliskTurrPos2 = turrDefault + blueObeliskTurrPos2;
                obeliskTurrPos3 = turrDefault + blueObeliskTurrPos3;
                turretShootPos = turrDefault + blueTurretShootPos;

                if (gamepad2.squareWasPressed()){
                    turret.pipelineSwitch(2);
                    robot.limelight.start();
                    drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

                    gamepad2.rumble(500);
                }
            }

            leave3Ball = drive.actionBuilder(autoStart)
                    .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));

            leaveFromShoot = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(xLeave, yLeave), Math.toRadians(hLeave));

            pickup3 = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(xStackPickupA, yStackPickupA), Math.toRadians(hStackPickupA))
                    .strafeToLinearHeading(new Vector2d(xStackPickupB, yStackPickupB), Math.toRadians(hStackPickupB),
                            new TranslationalVelConstraint(pickupStackSpeed));

            shoot3 = drive.actionBuilder(new Pose2d(xStackPickupB, yStackPickupB, Math.toRadians(hStackPickupB)))
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));

            pickupGate = drive.actionBuilder(new Pose2d(xShoot, yShoot, Math.toRadians(hShoot)))
                    .strafeToLinearHeading(new Vector2d(pickupGateX, pickupGateY), Math.toRadians(pickupGateH))
                    .waitSeconds(0.2)
                    .strafeToLinearHeading(new Vector2d(pickupGateXB, pickupGateYB), Math.toRadians(pickupGateHB))
                    .strafeToLinearHeading(new Vector2d(pickupGateXC, pickupGateYC), Math.toRadians(pickupGateHC),
                            new TranslationalVelConstraint(pickupGateSpeed));

            shootGate = drive.actionBuilder(new Pose2d(pickupGateX, pickupGateY, Math.toRadians(pickupGateH)))
                    .strafeToLinearHeading(new Vector2d(xShoot, yShoot), Math.toRadians(hShoot));

            limelightUsed = true;

            TELE.addData("Red?", redAlliance);
            TELE.addData("Turret Default", turrDefault);
            TELE.addData("Gate Cycle?", gatePickup);
            TELE.addData("Pickup Stack?", stack3);
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
                                xStackPickupB,
                                yStackPickupB,
                                hStackPickupB
                        ),
                        autoActions.detectObelisk(
                                intakeStackTime,
                                xStackPickupB,
                                yStackPickupB,
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

    void cycleGatePickupBalls(){
        Actions.runBlocking(
                new ParallelAction(
                        pickupGate.build(),
                        autoActions.intake(
                                intakeStackTime,
                                pickupGateX,
                                pickupGateY,
                                pickupGateH
                        ),
                        autoActions.detectObelisk(
                                intakeGateTime,
                                pickupGateX,
                                pickupGateY,
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