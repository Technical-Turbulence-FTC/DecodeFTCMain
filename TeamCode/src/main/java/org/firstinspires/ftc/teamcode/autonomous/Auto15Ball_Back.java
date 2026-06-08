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
public class Auto15Ball_Back extends LinearOpMode {
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
    public static double rapidWaitTime = 0.7;
    public static double rapidShootTime = 2.6;
    public static double loadPickupTime = 3;

    // Initialize path state machine
    private enum PathState {
        WAIT_VELOCITY, WAIT_SHOOT0,
        PICKUP3, DRIVE_SHOOT3, WAIT_SHOOT3,
        PICKUP_LOAD, DRIVE_SHOOT_LOAD, WAIT_SHOOT_LOAD,
        INTAKE_GATE, DRIVE_SHOOT_GATE, WAIT_SHOOT_GATE, LEAVE
    }
    PathState pathState = PathState.WAIT_VELOCITY;

    // Poses
    public static double startPoseX = 16, startPoseY = -64, startPoseH = 90;
    public static double pickup3ControlX = 12, pickup3ControlY = -37;
    public static double pickup3X = 61, pickup3Y = -37, pickup3H = 0;
    public static double shoot3X = 16, shoot3Y = -55, shoot3H = 0;
    public static double pickupLoadControlX = 21.23654066437572, pickupLoadControlY = -62.311626575028637;
    public static double pickupLoadX = 63, pickupLoadY = -63, pickupLoadH = 0;
    public static double shootLoadControlX = 21.23654066437572, shootLoadControlY = -62.311626575028637;
    public static double shootLoadX = 16, shootLoadY = -55, shootLoadH = 0;
    public static double intakeGateControl1X = 51.9656357388316, intakeGateControl1Y = -65.506277205040073;
    public static double intakeGateControl2X = 60.13459335624285, intakeGateControl2Y = -67.300458190148911;
    public static double intakeGateX = 63, intakeGateY = -35, intakeGateH = 90;
    public static double shootGateControlX = 53.8705612829324, shootGateControlY = -35.14501718213059;
    public static double shootGateX = 16, shootGateY = -55, shootGateH = 0;
    public static double leaveX = 36, leaveY = -58, leaveH = 0;
    double[] xPoses = {startPoseX, pickup3ControlX, pickup3X, shoot3X,
            pickupLoadControlX, pickupLoadX, shootLoadControlX, shootLoadX,
            intakeGateControl1X, intakeGateControl2X, intakeGateX, shootGateControlX, shootGateX, leaveX};
    double[] yPoses = {startPoseY, pickup3ControlY, pickup3Y, shoot3Y,
            pickupLoadControlY, pickupLoadY, shootLoadControlY, shootLoadY,
            intakeGateControl1Y, intakeGateControl2Y, intakeGateY, shootGateControlY, shootGateY, leaveY};
    double[] headings = {startPoseH, 0, pickup3H, shoot3H,
            0, pickupLoadH, 0, shootLoadH,
            0, 0, intakeGateH, 0, shootGateH, leaveH};
    Pose startPose, pickup3Control, pickup3, shoot3,
            pickupLoadControl, pickupLoad, shootLoadControl, shootLoad,
            intakeGateControl1, intakeGateControl2, intakeGate, shootGateControl, shootGate, leave;
    private void initializePoses(){
        startPose = new Pose(xPoses[0], yPoses[0], Math.toRadians(headings[0]));
        pickup3Control = new Pose(xPoses[1], yPoses[1]);
        pickup3 = new Pose(xPoses[2], yPoses[2], Math.toRadians(headings[2]));
        shoot3 = new Pose(xPoses[3], yPoses[3], Math.toRadians(headings[3]));
        pickupLoadControl = new Pose(xPoses[4], yPoses[4]);
        pickupLoad = new Pose(xPoses[5], yPoses[5], Math.toRadians(headings[5]));
        shootLoadControl = new Pose(xPoses[6], yPoses[6]);
        shootLoad = new Pose(xPoses[7], yPoses[7], Math.toRadians(headings[7]));
        intakeGateControl1 = new Pose(xPoses[8], yPoses[8]);
        intakeGateControl2 = new Pose(xPoses[9], yPoses[9]);
        intakeGate = new Pose(xPoses[10], yPoses[10], Math.toRadians(headings[10]));
        shootGateControl = new Pose(xPoses[11], yPoses[11]);
        shootGate = new Pose(xPoses[12], yPoses[12], Math.toRadians(headings[12]));
        leave = new Pose(xPoses[13], yPoses[13], Math.toRadians(headings[13]));
    }

    //Building Paths
    PathChain startPose_pickup3, pickup3_shoot3, shoot3_pickupLoad, pickupLoad_shootLoad,
            shootLoad_intakeGate, intakeGate_shootGate, shootGate_intakeGate, shootGate_leave;
    private void buildPaths(){
        startPose_pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, pickup3Control, pickup3))
                .setTangentHeadingInterpolation()
                .build();

        pickup3_shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, shoot3))
                .setLinearHeadingInterpolation(pickup3.getHeading(), shoot3.getHeading())
                .build();

        shoot3_pickupLoad = follower.pathBuilder()
                .addPath(new BezierCurve(shoot3, pickupLoadControl, pickupLoad))
                .setLinearHeadingInterpolation(shoot3.getHeading(), pickupLoad.getHeading())
                .build();

        pickupLoad_shootLoad = follower.pathBuilder()
                .addPath(new BezierCurve(pickupLoad, shootLoadControl, shootLoad))
                .setLinearHeadingInterpolation(pickupLoad.getHeading(), shootLoad.getHeading())
                .build();

        shootLoad_intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootLoad, intakeGateControl1, intakeGateControl2, intakeGate))
                .setTangentHeadingInterpolation()
                .build();

        intakeGate_shootGate = follower.pathBuilder()
                .addPath(new BezierCurve(intakeGate, shootGateControl, shootGate))
                .setLinearHeadingInterpolation(intakeGate.getHeading(), shootGate.getHeading())
                .build();

        shootGate_intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootGate, intakeGateControl1, intakeGateControl2, intakeGate))
                .setTangentHeadingInterpolation()
                .build();

        shootGate_leave = follower.pathBuilder()
                .addPath(new BezierLine(shootGate, leave))
                .setLinearHeadingInterpolation(shootGate.getHeading(), leave.getHeading())
                .build();
    }

    //Path State Machine
    private int startAuto = 0;
    private double timeStamp = 0;
    private void pathStateMachine(){
        double currentTime = (double) System.currentTimeMillis() / 1000;
        switch(pathState){
            case WAIT_VELOCITY:
                shooter.setFlywheelVelocity(3250);
                robot.setHoodPos(0.65);
                if (flywheel.getSteady()){
                    startAuto++;
                }
                if (startAuto > 6){
                    spindexer.startBackShooting();
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT0;
                }
                timeStamp = currentTime;
                break;
            case WAIT_SHOOT0:
                if (currentTime - timeStamp > rapidShootTime){
                    follower.followPath(startPose_pickup3, false);
                    pathState = PathState.PICKUP3;
                }
                break;
            case PICKUP3:
                if (!follower.isBusy()){
                    follower.followPath(pickup3_shoot3, true);
                    pathState = PathState.DRIVE_SHOOT3;
                    timeStamp = currentTime;
                }
                break;
            case DRIVE_SHOOT3:
                if (!follower.isBusy()  && currentTime - timeStamp > rapidWaitTime){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT3;
                    spindexer.startBackShooting();
                } else if (follower.isBusy()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT3:
                if (currentTime - timeStamp > rapidShootTime){
                    follower.followPath(shoot3_pickupLoad, false);
                    pathState = PathState.PICKUP_LOAD;
                    timeStamp = currentTime;
                }
                break;
            case PICKUP_LOAD:
                if (currentTime - timeStamp > loadPickupTime){
                    follower.followPath(pickupLoad_shootLoad, true);
                    pathState = PathState.DRIVE_SHOOT_LOAD;
                    timeStamp = currentTime;
                }
                break;
            case DRIVE_SHOOT_LOAD:
                if (!follower.isBusy()  && currentTime - timeStamp > rapidWaitTime){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT_LOAD;
                    spindexer.startBackShooting();
                } else if (follower.isBusy()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT_LOAD:
                if (currentTime - timeStamp > rapidShootTime){
                    follower.followPath(shootLoad_intakeGate, false);
                    pathState = PathState.INTAKE_GATE;
                    timeStamp = currentTime;
                }
                break;
            case INTAKE_GATE:
                if (!follower.isBusy()){
                    follower.followPath(intakeGate_shootGate, true);
                    pathState = PathState.DRIVE_SHOOT_GATE;
                    timeStamp = currentTime;
                }
                break;
            case DRIVE_SHOOT_GATE:
                if (!follower.isBusy()  && currentTime - timeStamp > rapidWaitTime){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT_GATE;
                    spindexer.startBackShooting();
                } else if (follower.isBusy()){
                    timeStamp = currentTime;
                }
                break;
            case WAIT_SHOOT_GATE:
                if (currentTime - timeStamp > rapidShootTime){
//                    follower.followPath(shootGate_intakeGate, false);
                    follower.followPath(shootGate_leave, true);
                    pathState = PathState.LEAVE;
                    timeStamp = currentTime;
                }
                break;
            case LEAVE:
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

            park.unpark();

            follower.update();

            if (gamepad1.squareWasPressed()){
                robot.setSpinPos(ServoPositions.spindexer_A2);
                robot.setRapidFireBlockerPos(ServoPositions.rapidFireBlocker_Open);
                robot.setSpindexBlockerPos(ServoPositions.spindexBlocker_Closed);
                robot.setTurretPos(Turret.neutralPosition);
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

            NewShooterTest.transferPower = -0.6;

            TELE.addData("Red Alliance?", Color.redAlliance);
            TELE.addData("Initialized Robot? (Don't change this until alliance is selected)", initializeRobot);
            TELE.addData("Start Pose", follower.getPose());
            TELE.addData("Current LL Pipeline", turret.pipeline());
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
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