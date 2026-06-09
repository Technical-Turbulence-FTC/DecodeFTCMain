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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
public class Auto21Ball_Front_Gate extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    Follower follower;
    MeasuringLoopTimes loopTimes;
    Shooter shooter;
    Turret turret;
    Flywheel flywheel;
    VelocityCommander commander;
    SpindexerTransferIntake spindexer;
    double runtime = 0;

    // Wait Times
    public static double rapidWaitTimeShoot0 = 0.2;
    public static double rapidWaitTimeShoot1 = 0.2;
    public static double rapidWaitTimeShoot2 = 0.2;
    public static double rapidWaitTimeShootGate = 0.4;
    public static double rapidShootTime = 0.4;
    public static double openGate1Time = 1.8;
    public static double openGate2Time = 1;
    public static double openGateWaitTimeMax = 3.5;
    public static int maxLoopCycles = 4;

    // Initialize path state machine
    private enum PathState {
        DRIVE_SHOOT0, WAIT_SHOOT0,
        PICKUP1, OPENGATE1, DRIVE_SHOOT1, WAIT_SHOOT1,
        PICKUP2, OPENGATE2, DRIVE_SHOOT2, WAIT_SHOOT2,
        INTAKE_GATE, DRIVE_SHOOT_GATE, WAIT_SHOOT_GATE, DRIVE_SHOOT_LEAVE, WAIT_SHOOT_LEAVE
    }
    PathState pathState = PathState.DRIVE_SHOOT0;

    // Poses
    public static double startPoseX = 55, startPoseY = 39, startPoseH = 0;
    public static double shoot0X = 25, shoot0Y = 18, shoot0H = 0;
    public static double pickup1ControlX = 29.53321878579611, pickup1ControlY = 9.84077892325314;
    public static double pickup1X = 50, pickup1Y = 10, pickup1H = 0;
    public static double openGate1ControlX = 43.82989690721648, openGate1ControlY = 3.86540664375714;
    public static double openGate1X = 59, openGate1Y = 2, openGate1H = 0;
    public static double shoot1ControlX = 40, shoot1ControlY = 3;
    public static double shoot1X = 25, shoot1Y = 11, shoot1H = -30;
    public static double pickup2ControlX = 18, pickup2ControlY = -18;
    public static double pickup2X = 61, pickup2Y = -14.5, pickup2H = 0;
    public static double openGate2ControlX = 45.9782359679267, openGate2ControlY = -15.106643757159245;
    public static double openGate2X = 57, openGate2Y = -8, openGate2H = 0;
    public static double shoot2ControlX = 57, shoot2ControlY = -8;
    public static double shoot2X = 16, shoot2Y = 4, shoot2H = -30;
    public static double intakeGateControlX = 60, intakeGateControlY = -12;
    public static double toGateX = 60, toGateY = -10;
    public static double intakeGateX = 62, intakeGateY = -12.5, intakeGateH = 25;
    public static double shootGateControlX = 40, shootGateControlY = -5;
    public static double shootGateX = 16, shootGateY = 4, shootGateH = -30;
    public static double shootLeaveControlX = 56, shootLeaveControlY = -10;
    public static double shootLeaveX = 16, shootLeaveY = 36, shootLeaveH = -50;
    public static double leaveX = 45, leaveY = 10, leaveH = 0;
    public static double awayFromGateX = 40, awayFromGateY = -12, awayFromGateH = 0;
    double[] xPoses = {startPoseX, shoot0X,
            pickup1ControlX, pickup1X, openGate1ControlX, openGate1X, shoot1ControlX, shoot1X,
            pickup2ControlX, pickup2X, openGate2ControlX, openGate2X, shoot2ControlX, shoot2X,
            intakeGateControlX, intakeGateX, shootGateControlX, shootGateX,
            shootLeaveControlX, shootLeaveX, leaveX, awayFromGateX, toGateX};
    double[] yPoses = {startPoseY, shoot0Y,
            pickup1ControlY, pickup1Y, openGate1ControlY, openGate1Y, shoot1ControlY, shoot1Y,
            pickup2ControlY, pickup2Y, openGate2ControlY, openGate2Y, shoot2ControlY, shoot2Y,
            intakeGateControlY, intakeGateY, shootGateControlY, shootGateY,
            shootLeaveControlY, shootLeaveY, leaveY, awayFromGateY, toGateY};
    double[] headings = {startPoseH, shoot0H,
            0, pickup1H, 0, openGate1H, 0, shoot1H,
            0, pickup2H, 0, openGate2H, 0, shoot2H,
            0, intakeGateH, 0, shootGateH,
            0, shootLeaveH, leaveH, awayFromGateH, 0};
    Pose startPose, shoot0,
            pickup1Control, pickup1, openGate1Control, openGate1, shoot1Control, shoot1,
            pickup2Control, pickup2, openGate2Control, openGate2, shoot2Control, shoot2,
            intakeGateControl, intakeGate, shootGateControl, shootGate,
            shootLeaveControl, shootLeave, leave, awayFromGate, toGate;
    private void initializePoses(){
        startPose = new Pose(xPoses[0], yPoses[0], Math.toRadians(headings[0]));
        shoot0 = new Pose(xPoses[1], yPoses[1], Math.toRadians(headings[1]));
        pickup1Control = new Pose(xPoses[2], yPoses[2]);
        pickup1 = new Pose(xPoses[3], yPoses[3], Math.toRadians(headings[3]));
        openGate1Control = new Pose(xPoses[4], yPoses[4]);
        openGate1 = new Pose(xPoses[5], yPoses[5], Math.toRadians(headings[5]));
        shoot1Control = new Pose(xPoses[6], yPoses[6]);
        shoot1 = new Pose(xPoses[7], yPoses[7], Math.toRadians(headings[7]));
        pickup2Control = new Pose(xPoses[8], yPoses[8]);
        pickup2 = new Pose(xPoses[9], yPoses[9], Math.toRadians(headings[9]));
        openGate2Control = new Pose(xPoses[10], yPoses[10]);
        openGate2 = new Pose(xPoses[11], yPoses[11], Math.toRadians(headings[11]));
        shoot2Control = new Pose(xPoses[12], yPoses[12]);
        shoot2 = new Pose(xPoses[13], yPoses[13], Math.toRadians(headings[13]));
        intakeGateControl = new Pose(xPoses[14], yPoses[14]);
        intakeGate = new Pose(xPoses[15], yPoses[15], Math.toRadians(headings[15]));
        shootGateControl = new Pose(xPoses[16], yPoses[16]);
        shootGate = new Pose(xPoses[17], yPoses[17], Math.toRadians(headings[17]));
        shootLeaveControl = new Pose(xPoses[18], yPoses[18]);
        shootLeave = new Pose(xPoses[19], yPoses[19], Math.toRadians(headings[19]));
        leave = new Pose(xPoses[20], yPoses[20], Math.toRadians(headings[20]));
        awayFromGate = new Pose(xPoses[21], yPoses[21], Math.toRadians(headings[21]));
        toGate = new Pose(xPoses[22], yPoses[22]);
    }

    //Building Paths
    PathChain startPose_shoot0, shoot0_pickup1, pickup1_openGate1, openGate1_shoot1,
        shoot1_pickup2, pickup2_openGate2, openGate2_shoot2, shootGate_intakeGate,
        shoot2_intakeGate, intakeGate_shootGate, intakeGate_shootLeave, shootGate_leave, intakeGate_awayFromGate;
    private void buildPaths(){
        startPose_shoot0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot0))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot0.getHeading())
                .build();

        shoot0_pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot0, pickup1Control, pickup1))
                .setLinearHeadingInterpolation(shoot0.getHeading(), pickup1.getHeading())
                .build();

        pickup1_openGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1, openGate1Control, openGate1))
                .setLinearHeadingInterpolation(pickup1.getHeading(), openGate1.getHeading())
                .build();

        openGate1_shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(openGate1, shoot1Control, shoot1))
                .setLinearHeadingInterpolation(openGate1.getHeading(), shoot1.getHeading())
                .build();

        shoot1_pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1, pickup2Control, pickup2))
                .setLinearHeadingInterpolation(shoot1.getHeading(), pickup2.getHeading())
                .build();

        pickup2_openGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2, openGate2Control, openGate2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), openGate2.getHeading())
                .build();

        openGate2_shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(openGate2, shoot2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shoot2_intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(shoot2, intakeGateControl, toGate))
                .setLinearHeadingInterpolation(shoot2.getHeading(), intakeGate.getHeading())
                .build();

        intakeGate_shootGate = follower.pathBuilder()
                .addPath(new BezierCurve(intakeGate, shootGateControl, shootGate))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shootGate_intakeGate = follower.pathBuilder()
                .addPath(new BezierLine(shootGate, intakeGate))
                .setLinearHeadingInterpolation(shootGate.getHeading(), intakeGate.getHeading())
                .build();

        intakeGate_shootLeave = follower.pathBuilder()
                .addPath(new BezierCurve(intakeGate, pickup2Control, shootLeave))
                .setLinearHeadingInterpolation(intakeGate.getHeading(), shootLeave.getHeading())
                .build();

        shootGate_leave = follower.pathBuilder()
                .addPath(new BezierLine(shootGate, leave))
                .setLinearHeadingInterpolation(shootGate.getHeading(), leave.getHeading())
                .build();

        intakeGate_awayFromGate = follower.pathBuilder()
                .addPath(new BezierLine(intakeGate, awayFromGate))
                .setLinearHeadingInterpolation(intakeGate.getHeading(), awayFromGate.getHeading())
                .build();
    }

    //Path State Machine
    private boolean startAuto = true;
    private double timeStamp = 0;
    private int cycle = 0;
    boolean toGateBool = false;
    private void pathStateMachine(){
        double currentTime = (double) System.currentTimeMillis() / 1000;
        switch(pathState){
            case DRIVE_SHOOT0:
                spindexer.setSpindexerMode(SpindexerTransferIntake.SpindexerMode.RAPID);
                shooter.setFlywheelVelocity(2300);
                robot.setHoodPos(0.68);
                if (startAuto){
                    follower.followPath(startPose_shoot0, false);
                    startAuto = false;
                }
                if ((follower.atParametricEnd() || !follower.isBusy())  && currentTime - timeStamp > rapidWaitTimeShoot0){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT0;
                    spindexer.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
                } else if (follower.isBusy() && !follower.atParametricEnd()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT0:
                if (currentTime - timeStamp > rapidShootTime ||
                        (spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.OPEN_GATE &&
                                spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.SHOOT)){
                    follower.followPath(shoot0_pickup1, false);
                    pathState = PathState.PICKUP1;
                }
                break;
            case PICKUP1:
                if (!follower.isBusy()){
                    follower.followPath(pickup1_openGate1, false);
                    pathState = PathState.OPENGATE1;
                    timeStamp = currentTime;
                }
                break;
            case OPENGATE1:
                if (currentTime - timeStamp > openGate1Time){
                    follower.followPath(openGate1_shoot1, false);
                    pathState = PathState.DRIVE_SHOOT1;
                    timeStamp = currentTime;
                }
                break;
            case DRIVE_SHOOT1:
                if (!follower.isBusy()  && currentTime - timeStamp > rapidWaitTimeShoot1){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT1;
                    spindexer.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
                } else if (follower.isBusy()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT1:
                if (currentTime - timeStamp > rapidShootTime ||
                        (spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.OPEN_GATE &&
                                spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.SHOOT)){
                    follower.followPath(shoot1_pickup2, false);
                    pathState = PathState.PICKUP2;
                }
                break;
            case PICKUP2:
                if (!follower.isBusy()){
                    shooter.setFlywheelVelocity(2500);
                    robot.setHoodPos(0.6);
                    follower.followPath(pickup2_openGate2, false);
                    pathState = PathState.OPENGATE2;
                    timeStamp = currentTime;
                }
                break;
            case OPENGATE2:
                if (currentTime - timeStamp > openGate2Time){
                    follower.followPath(openGate2_shoot2, false);
                    pathState = PathState.DRIVE_SHOOT2;
                    timeStamp = currentTime;
                }
                break;
            case DRIVE_SHOOT2:
                if (!follower.isBusy()  && currentTime - timeStamp > rapidWaitTimeShoot2){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT2;
                    spindexer.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
                } else if (follower.isBusy()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT2:
                if (currentTime - timeStamp > rapidShootTime ||
                        (spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.OPEN_GATE &&
                                spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.SHOOT)){
                    follower.followPath(shoot2_intakeGate, false);
                    pathState = PathState.INTAKE_GATE;
                    timeStamp = currentTime;
                    toGateBool = true;
                }
                break;
            case INTAKE_GATE:
                if ((currentTime - timeStamp > openGateWaitTimeMax)){
                    if (getRuntime() - runtime > 27){
                        follower.followPath(intakeGate_awayFromGate, true);
                        pathState = PathState.WAIT_SHOOT_LEAVE;
                    } else if (getRuntime() - runtime > 22 || cycle >= maxLoopCycles - 1){
                        follower.followPath(intakeGate_shootLeave, true);
                        pathState = PathState.DRIVE_SHOOT_LEAVE;
                        shooter.setFlywheelVelocity(2300);
                        robot.setHoodPos(0.68);
                    } else {
                        follower.followPath(intakeGate_shootGate, false);
                        pathState = PathState.DRIVE_SHOOT_GATE;
                    }
                    timeStamp = currentTime;
                }
                // TODO: add logic to shoot gate
                break;
            case DRIVE_SHOOT_GATE:
                if (!follower.isBusy()  && currentTime - timeStamp > rapidWaitTimeShootGate){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT_GATE;
                    spindexer.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
                } else if (follower.isBusy()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT_GATE:
                if (currentTime - timeStamp > rapidShootTime ||
                        (spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.OPEN_GATE &&
                                spindexer.getRapidState() != SpindexerTransferIntake.RapidMode.SHOOT)){
                    cycle++;
                    if (getRuntime() - runtime > 24 || cycle >= maxLoopCycles){
                        follower.followPath(shootGate_leave, true);
                        pathState = PathState.WAIT_SHOOT_LEAVE;
                    } else {
                        follower.followPath(shootGate_intakeGate, false);
                        pathState = PathState.INTAKE_GATE;
                        toGateBool = true;
                    }
                    timeStamp = currentTime;
                }
                break;
            case DRIVE_SHOOT_LEAVE:
                if (!follower.isBusy()  && currentTime - timeStamp > rapidWaitTimeShootGate){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT_LEAVE;
                    spindexer.setRapidMode(SpindexerTransferIntake.RapidMode.OPEN_GATE);
                } else if (follower.isBusy()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT_LEAVE:
                // add line here to say "done auto"
                break;
            default:
                break;
        }
        TELE.addData("Moving?", follower.isBusy());
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
        boolean startAuto = true;

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
                turret.switchPipeline(Turret.PipelineMode.TRACKING);
                robot.limelight.start();
                limelightUsed = true;
                park.unpark();
            }

            NewShooterTest.transferPower = -0.8;

            TELE.addData("Red Alliance?", Color.redAlliance);
            TELE.addData("Initialized Robot? (Don't change this until alliance is selected)", initializeRobot);
            TELE.addData("Start Pose", follower.getPose());
            TELE.addData("Current LL Pipeline", turret.pipeline());
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            if (startAuto){
                runtime = getRuntime();
                startAuto = false;
            }

            park.unpark();
            robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Open);

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