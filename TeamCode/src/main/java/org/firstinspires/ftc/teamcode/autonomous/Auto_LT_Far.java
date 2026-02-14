package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Back_Poses.*;
import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinEndPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall2;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall3b;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Targeting.turretInterpolate;
import static org.firstinspires.ftc.teamcode.utils.Turret.turrDefault;

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

import org.firstinspires.ftc.teamcode.autonomous.actions.AutoActions;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

import java.util.Objects;

@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class Auto_LT_Far extends LinearOpMode {
    public static double shoot0Vel = 3300, shoot0Hood = 0.48;
    public static double autoSpinStartPos = 0.2;
    public static double shoot0SpinSpeedIncrease = 0.015;
    public static double shoot0XTolerance = 1.0;
    public static double redTurretShootPos = 0.05;
    public static double blueTurretShootPos = -0.05;
    public static int fwdTime = 200, strafeTime = 2300;
    double xLeave, yLeave, hLeave;
    public static int sleepTime = 1300;
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
    double firstSpindexShootPos = autoSpinStartPos;
    boolean shootForward = true;
    double xShoot, yShoot, hShoot;
    private int driverSlotGreen = 0;
    private int passengerSlotGreen = 0;
    int rearSlotGreen = 0;
    int mostGreenSlot = 0;
    double pickupGateX = 0, pickupGateY = 0, pickupGateH = 0;
    double pickupZoneX = 0, pickupZoneY = 0, pickupZoneH = 0;
    public static double firstShootTime = 0.3;
    public static double flywheel0Time = 3.5;
    public static double shoot0Time = 2;
    boolean gatePickup = false;
    boolean stack3 = true;
    double xStackPickupA, yStackPickupA, hStackPickupA;
    double xStackPickupB, yStackPickupB, hStackPickupB;
    public static int pickupStackSpeed = 12;
    int prevMotif = 0;
    public static double spindexerSpeedIncrease = 0.005;


    // initialize path variables here
    TrajectoryActionBuilder leave3Ball = null;
    TrajectoryActionBuilder leaveFromShoot = null;
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

        robot.limelight.start();

        robot.limelight.pipelineSwitch(1);

        turret = new Turret(robot, TELE, robot.limelight);

        turret.setTurret(turrDefault);

        drive = new MecanumDrive(hardwareMap, autoStart);

        autoActions = new AutoActions(robot, drive, TELE, servos, flywheel, spindexer, targeting, targetingSettings, turret);

        servos.setSpinPos(autoSpinStartPos);

        servos.setTransferPos(transferServo_out);

        while (opModeInInit()) {

            // Recalibration in initialization
            drive.updatePoseEstimate();
            if (gamepad2.triangle) {
                autoStart = drive.localizer.getPose(); // use this position as starting position
                gamepad2.rumble(1000);
            }

            if (gamepad2.squareWasPressed()){
                robot.limelight.start();
                robot.limelight.pipelineSwitch(1);
                gamepad2.rumble(500);
            }

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

                turretShootPos = turrDefault + redTurretShootPos;
            } else {
                robot.light.setPosition(0.6);

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

                turretShootPos = turrDefault + blueTurretShootPos;
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

            robot.transfer.setPower(1);

            startAuto();

            if (stack3){
                //cycleStackFar();
            }

            //TODO: insert code here for gate auto

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
                                false
                        ),
                        autoActions.shootAllAuto(shootAllTime, spindexerSpeedIncrease)
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
                                0.501,
                                0.501,
                                false
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

//    void cycleStackFar(){
//        Actions.runBlocking(
//                new ParallelAction(
//                        pickup3.build(),
//                        manageShooterAuto(
//                                intake3Time,
//                                0.501,
//                                0.501,
//                                0.501,
//                                0.501
//                        ),
//                        intake(intake3Time),
//                        detectObelisk(
//                                intake3Time,
//                                0.501,
//                                0.501,
//                                0.501,
//                                0.501,
//                                obeliskTurrPos3
//                        )
//
//                )
//        );
//
//        motif = turret.getObeliskID();
//
//        if (motif == 0) motif = prevMotif;
//        prevMotif = motif;
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        manageFlywheelAuto(
//                                shoot3Time,
//                                0.501,
//                                0.501,
//                                0.501,
//                                0.501
//                        ),
//                        shoot3.build(),
//                        prepareShootAll(colorSenseTime, shoot3Time, motif)
//                )
//        );
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        manageShooterAuto(
//                                finalShootAllTime,
//                                0.501,
//                                0.501,
//                                0.501,
//                                0.501
//                        ),
//                        shootAllAuto(finalShootAllTime, finalSpindexerSpeedIncrease)
//                )
//
//        );
//    }
}