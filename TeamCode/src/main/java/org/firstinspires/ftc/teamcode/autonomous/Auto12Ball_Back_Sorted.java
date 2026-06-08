package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.utils.Turret.limelightUsed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.Color;
import org.firstinspires.ftc.teamcode.constants.ServoPositions;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teleop.TeleopV4;
import org.firstinspires.ftc.teamcode.tests.NewShooterTest;
import org.firstinspires.ftc.teamcode.utilsv2.*;
import org.firstinspires.ftc.teamcode.utils.MeasuringLoopTimes;

import java.util.List;

@Config
@Autonomous (preselectTeleOp = "TeleopV4")
public class Auto12Ball_Back_Sorted extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    Follower follower;
    MeasuringLoopTimes loopTimes;
    Shooter shooter;
    Turret turret;
    Flywheel flywheel;
    VelocityCommander commander;
    SpindexerTransferIntake spindexer;

    // Wait Times
    public static double sortedShootTime = 2.6;
    public static double rapidWaitTime = 0.5;
    public static double rapidShootTime = 0.8;
    public static double openGateTime = 2.5;
    public static double pushTime = 2;

    // Extra Variables
    public static double intakePower = 0.5;
    double shootX, shootY, shootH;

    // Initialize path state machine
    private enum PathState {
        PUSHBOT, DRIVE_SHOOT0, WAIT_SHOOT0,
        PICKUP1, OPENGATE, DRIVE_SHOOT1, WAIT_SHOOT1,
        DRIVE_PICKUP2, PICKUP2, DRIVE_SHOOT2, WAIT_SHOOT2,
        DRIVE_PICKUP3, PICKUP3, DRIVE_SHOOT3, WAIT_SHOOT3
    }
    PathState pathState = PathState.PUSHBOT;

    // Poses
    public static double startPoseX = 12, startPoseY = -64, startPoseH = 90;
    public static double pushBotX = 19, pushBotY = -63, pushBotH = 100;
    public static double shoot0ControlX = 16.29667812142038, shoot0ControlY = -19.67493699885454;
    public static double shoot0X = 19, shoot0Y = 10, shoot0H = 0;
    public static double pickup1X = 54, pickup1Y = 10, pickup1H = 0;
    public static double openGateControlX = 37.184421534937, openGateControlY = 2.24455899198165;
    public static double openGateX = 59, openGateY = 2, openGateH = 0;
    public static double shoot1ControlX = 40, shoot1ControlY = 3;
    public static double shoot1X = 19, shoot1Y = 10, shoot1H = 0;
    public static double drivePickup2X = 26, drivePickup2Y = -14, drivePickup2H = 0;
    public static double pickup2X = 61, pickup2Y = -14.5, pickup2H = 0;
    public static double shoot2ControlX = 30, shoot2ControlY = -9;
    public static double shoot2X = 19, shoot2Y = 10, shoot2H = 0;
    public static double drivePickup3X = 26, drivePickup3Y = -37.5, drivePickup3H = 0;
    public static double pickup3X = 61, pickup3Y = -37.5, pickup3H = 0;
    public static double shoot3ControlX = 25.62371134020621, shoot3ControlY = -38.813287514318446;
    public static double shoot3X = 12, shoot3Y = 40, shoot3H = -90;
    double[] xPoses = {startPoseX, pushBotX, shoot0ControlX, shoot0X,
            pickup1X, openGateControlX, openGateX, shoot1ControlX, shoot1X,
            drivePickup2X, pickup2X, shoot2ControlX, shoot2X,
            drivePickup3X, pickup3X, shoot3ControlX, shoot3X};
    double[] yPoses = {startPoseY, pushBotY, shoot0ControlY, shoot0Y,
            pickup1Y, openGateControlY, openGateY, shoot1ControlY, shoot1Y,
            drivePickup2Y, pickup2Y, shoot2ControlY, shoot2Y,
            drivePickup3Y, pickup3Y, shoot3ControlY, shoot3Y};
    double[] headings = {startPoseH, pushBotH, 0, shoot0H,
            pickup1H, 0, openGateH, 0, shoot1H,
            drivePickup2H, pickup2H, 0, shoot2H,
            drivePickup3H, pickup3H, 0, shoot3H};
    Pose startPose, pushBot, shoot0Control, shoot0,
            pickup1, openGateControl, openGate, shoot1Control, shoot1,
            drivePickup2, pickup2, shoot2Control, shoot2,
            drivePickup3, pickup3, shoot3Control, shoot3;
    private void initializePoses(){
        startPose = new Pose(xPoses[0], yPoses[0], Math.toRadians(headings[0]));
        pushBot = new Pose(xPoses[1], yPoses[1], Math.toRadians(headings[1]));
        shoot0Control = new Pose(xPoses[2], yPoses[2]);
        shoot0 = new Pose(xPoses[3], yPoses[3], Math.toRadians(headings[3]));
        pickup1 = new Pose(xPoses[4], yPoses[4], Math.toRadians(headings[4]));
        openGateControl = new Pose(xPoses[5], yPoses[5]);
        openGate = new Pose(xPoses[6], yPoses[6], Math.toRadians(headings[6]));
        shoot1Control = new Pose(xPoses[7], yPoses[7]);
        shoot1 = new Pose(xPoses[8], yPoses[8], Math.toRadians(headings[8]));
        drivePickup2 = new Pose(xPoses[9], yPoses[9], Math.toRadians(headings[9]));
        pickup2 = new Pose(xPoses[10], yPoses[10], Math.toRadians(headings[10]));
        shoot2Control = new Pose(xPoses[11], yPoses[11]);
        shoot2 = new Pose(xPoses[12], yPoses[12], Math.toRadians(headings[12]));
        drivePickup3 = new Pose(xPoses[13], yPoses[13], Math.toRadians(headings[13]));
        pickup3 = new Pose(xPoses[14], yPoses[14], Math.toRadians(headings[14]));
        shoot3Control = new Pose(xPoses[15], yPoses[15]);
        shoot3 = new Pose(xPoses[16], yPoses[16], Math.toRadians(headings[16]));
    }

    //Building Paths
    PathChain startPose_pushBot, pushBot_shoot0,
            shoot0_pickup1, pickup1_openGate, openGate_shoot1,
            shoot1_drivePickup2, drivePickup2_pickup2, pickup2_shoot2,
            shoot2_drivePickup3, drivePickup3_pickup3, pickup3_shoot3;
    private void buildPaths(){
        startPose_pushBot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pushBot))
                .setLinearHeadingInterpolation(startPose.getHeading(), pushBot.getHeading())
                .build();

        pushBot_shoot0 = follower.pathBuilder()
                .addPath(new BezierCurve(pushBot, shoot0Control, shoot0))
                .setLinearHeadingInterpolation(pushBot.getHeading(), shoot0.getHeading())
                .build();

        shoot0_pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot0, pickup1))
                .setLinearHeadingInterpolation(shoot0.getHeading(), pickup1.getHeading())
                .build();

        pickup1_openGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1, openGateControl, openGate))
                .setLinearHeadingInterpolation(pickup1.getHeading(), openGate.getHeading())
                .build();

        openGate_shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(openGate, shoot1Control, shoot1))
                .setLinearHeadingInterpolation(openGate.getHeading(), shoot1.getHeading())
                .build();

        shoot1_drivePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(shoot1, drivePickup2))
                .setLinearHeadingInterpolation(shoot1.getHeading(), drivePickup2.getHeading())
                .build();

        drivePickup2_pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(drivePickup2, pickup2))
                .setLinearHeadingInterpolation(drivePickup2.getHeading(), pickup2.getHeading())
                .build();

        pickup2_shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2, shoot2Control, shoot2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shoot2.getHeading())
                .build();

        shoot2_drivePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, drivePickup3))
                .setLinearHeadingInterpolation(shoot2.getHeading(), drivePickup3.getHeading())
                .build();

        drivePickup3_pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(drivePickup3, pickup3))
                .setLinearHeadingInterpolation(drivePickup3.getHeading(), pickup3.getHeading())
                .build();

        pickup3_shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3, shoot3Control, shoot3))
                .setLinearHeadingInterpolation(pickup3.getHeading(), shoot3.getHeading())
                .build();
    }

    //Path State Machine
    private boolean startAuto = true;
    private double timeStamp = 0;
    private void pathStateMachine(){
        double currentTime = (double) System.currentTimeMillis() / 1000;
        switch(pathState){
            case PUSHBOT:
                spindexer.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);
                shooter.setFlywheelVelocity(2400);
                robot.setHoodPos(0.64);
                if (startAuto){
                    follower.followPath(startPose_pushBot, false);
                    startAuto = false;
                    timeStamp = currentTime;
                    shootX = shoot0X;
                    shootY = shoot0Y;
                    shootH = shoot0H;
                }
                if (!follower.isBusy() || currentTime - timeStamp > pushTime){
                    follower.followPath(pushBot_shoot0, true);
                    pathState = PathState.DRIVE_SHOOT0;
                }
                break;
            case DRIVE_SHOOT0:
                if (!follower.isBusy()  && currentTime - timeStamp > rapidWaitTime){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT0;
                    spindexer.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
                } else if (follower.isBusy()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT0:
                if (currentTime - timeStamp > rapidShootTime ||
                        (spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.OPEN_GATE &&
                                spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.SHOOT)){
                    follower.followPath(shoot0_pickup1, intakePower, false);
                    pathState = PathState.PICKUP1;
                    spindexer.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.SORTED);
                }
                break;
            case PICKUP1:
                if (!follower.isBusy()){
                    follower.followPath(pickup1_openGate, false);
                    pathState = PathState.OPENGATE;
                    shootX = shoot1X;
                    shootY = shoot1Y;
                    shootH = shoot1H;
                    timeStamp = currentTime;
                    shooter.setFlywheelVelocity(2300);
                    robot.setHoodPos(0.68);
                }
                break;
            case OPENGATE:
                if (currentTime - timeStamp > openGateTime){
                    follower.followPath(openGate_shoot1, true);
                    pathState = PathState.DRIVE_SHOOT1;
                }
                break;
            case DRIVE_SHOOT1:
                if (!follower.isBusy()){
                    pathState = PathState.WAIT_SHOOT1;
                    timeStamp = currentTime;
                    spindexer.startSortedShoot();
                }
                break;
            case WAIT_SHOOT1:
                if (currentTime - timeStamp > sortedShootTime){
                    follower.followPath(shoot1_drivePickup2, true);
                    pathState = PathState.DRIVE_PICKUP2;
                }
                break;
            case DRIVE_PICKUP2:
                if (!follower.isBusy()) {
                    follower.followPath(drivePickup2_pickup2, intakePower, false);
                    pathState = PathState.PICKUP2;
                }
                break;
            case PICKUP2:
                if (!follower.isBusy()){
                    follower.followPath(pickup2_shoot2, true);
                    pathState = PathState.DRIVE_SHOOT2;
                }
                shootX = shoot2X;
                shootY = shoot2Y;
                shootH = shoot2H;
                break;
            case DRIVE_SHOOT2:
                if (!follower.isBusy()){
                    pathState = PathState.WAIT_SHOOT2;
                    timeStamp = currentTime;
                    spindexer.startSortedShoot();
                }
                break;
            case WAIT_SHOOT2:
                if (currentTime - timeStamp > sortedShootTime){
                    follower.followPath(shoot2_drivePickup3, true);
                    pathState = PathState.DRIVE_PICKUP3;
                }
                break;
            case DRIVE_PICKUP3:
                if (!follower.isBusy()){
                    shooter.setFlywheelVelocity(2200);
                    robot.setHoodPos(0.75);
                    follower.followPath(drivePickup3_pickup3, intakePower, false);
                    pathState = PathState.PICKUP3;
                }
                break;
            case PICKUP3:
                if (!follower.isBusy()){
                    follower.followPath(pickup3_shoot3, true);
                    pathState = PathState.DRIVE_SHOOT3;
                }
                shootX = shoot3X;
                shootY = shoot3Y;
                shootH = shoot3H;
                break;
            case DRIVE_SHOOT3:
                if (!follower.isBusy()){
                    pathState = PathState.WAIT_SHOOT3;
                    spindexer.startSortedShoot();
                }
                break;
            case WAIT_SHOOT3:
                // add line here to say "done auto"
                break;
            default:
                break;
        }
        TELE.update(); // use for debugging
    }

    // Used for changing alliance
    private double adjustXPoseBasedOnAlliance(double pose) {return -pose;}
    private double adjustHeadingBasedOnAlliance(double heading){
        heading = 180 - heading;
        while (heading > 180) {heading-=360;}
        while (heading <= -180) {heading+=360;}
        return heading;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.resetInstance();
        robot = Robot.getInstance(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        sleep(1000);
        follower.setStartingPose(new Pose(0,0,0));
        loopTimes = new MeasuringLoopTimes();
        loopTimes.init();
        turret = new Turret(robot);
        flywheel = new Flywheel(robot);
        commander = new VelocityCommander();
        shooter = new Shooter(robot, TELE, follower, Color.redAlliance, turret, flywheel, commander);
        spindexer = new SpindexerTransferIntake(robot, TELE, commander);
        ParkTilter park = new ParkTilter(robot);

        boolean initializeRobot = false;
        while (opModeInInit()){
            follower.update();

            if (gamepad1.squareWasPressed()){
                robot.setSpinPos(ServoPositions.spindexer_A2);
                robot.setRapidFireBlockerPos(ServoPositions.rapidFireBlocker_Closed);
                robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Open);
            }

            if (gamepad1.crossWasPressed() && !initializeRobot){
                Color.redAlliance = !Color.redAlliance;
                shooter.setRedAlliance(Color.redAlliance);
            }

            if (!initializeRobot){
                if ((Color.redAlliance && xPoses[0] < 0)
                    || (!Color.redAlliance && xPoses[0] > 0)){
                    for (int i = 0; i < xPoses.length; i++) {xPoses[i] = adjustXPoseBasedOnAlliance(xPoses[i]);}
                    for (int i = 0; i < headings.length; i++) {headings[i] = adjustHeadingBasedOnAlliance(headings[i]);}
                }
            }

            if (gamepad1.triangleWasPressed()){
                initializeRobot = true;
                initializePoses();
                follower.setPose(startPose);
                buildPaths();
                sleep(2000);
                turret.switchPipeline(Turret.PipelineMode.OBELISK);
                robot.limelight.start();
                limelightUsed = true;
                park.unpark();
            }

            NewShooterTest.transferPower = -0.8;

            if (initializeRobot){
                //add obelisk read here
                shooter.setState(Shooter.ShooterState.READ_OBELISK);
                int ID = turret.getObeliskID();
                spindexer.setDesiredPatternAuto(ID);
                TELE.addData("ID", ID);
                shooter.update(robot.voltage.getVoltage());
            }

            TELE.addData("Red Alliance?", Color.redAlliance);
            TELE.addData("Initialized Robot? (Don't change this until alliance is selected)", initializeRobot);
            TELE.addData("Start Pose", follower.getPose());
            TELE.addData("Current LL Pipeline", turret.pipeline());
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            robot.setRapidFireBlockerPos(ServoPositions.rapidFireBlocker_Closed);
            park.unpark();

            shooter.setState(Shooter.ShooterState.MANUAL_FLYWHEEL_TRACK_TURR);
            shooter.update(robot.voltage.getVoltage());

            if (!isStopRequested()){
                follower.update();
            }
            pathStateMachine();
            TeleopV4.teleStart = follower.getPose();

            spindexer.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            loopTimes.loop();

            TELE.addData("Avg Loop Time", loopTimes.getAvgLoopTime());
            TELE.addData("Min Loop Time", loopTimes.getMinLoopTimeOneMin());
            TELE.addData("Max Loop Time", loopTimes.getMaxLoopTimeOneMin());
            TELE.addData("X:", TeleopV4.teleStart.getX());
            TELE.addData("Y:", TeleopV4.teleStart.getY());
            TELE.addData("H:", TeleopV4.teleStart.getHeading());
            TELE.update();
        }
    }
}