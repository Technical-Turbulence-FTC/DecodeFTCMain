package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.utils.Turret.limelightUsed;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseH;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseY;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseX;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utilsv2.Flywheel;
import org.firstinspires.ftc.teamcode.utils.MeasuringLoopTimes;
import org.firstinspires.ftc.teamcode.utilsv2.Robot;
import org.firstinspires.ftc.teamcode.utilsv2.Turret;

import java.util.List;

@Config
@Autonomous (preselectTeleOp = "TeleopV4")
public class Auto12Ball_Back_Sorted extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    Follower follower;
    MeasuringLoopTimes loopTimes;

    // Wait Times
    public static double shootTime = 2;
    public static double openGateTime = 1.5;

    // Extra Variables
    public static double intakePower = 0.3;
    double shootX, shootY, shootH;

    // Initialize path state machine
    private enum PathState {
        PUSHBOT, DRIVE_SHOOT0, WAIT_SHOOT0,
        PICKUP1, DRIVE_OPENGATE, OPENGATE, DRIVE_SHOOT1, WAIT_SHOOT1,
        DRIVE_PICKUP2, PICKUP2, DRIVE_SHOOT2, WAIT_SHOOT2,
        DRIVE_PICKUP3, PICKUP3, DRIVE_SHOOT3, WAIT_SHOOT3
    }
    PathState pathState = PathState.PUSHBOT;

    // Poses
    public static double startPoseX = 84, startPoseY = 7, startPoseH = 90;
    public static double pushBotX = 94, pushBotY = 9, pushBotH = 100;
    public static double shoot0ControlX = 88.29667812142038, shoot0ControlY = 52.03493699885454;
    public static double shoot0X = 91, shoot0Y = 80, shoot0H = 0;
    public static double pickup1ControlX = 109.29381443298968, pickup1ControlY = 82.70618556701031;
    public static double pickup1X = 126, pickup1Y = 82, pickup1H = 0;
    public static double openGateControlX = 109.184421534937, openGateControlY = 74.24455899198165;
    public static double openGateX = 129, openGateY = 74, openGateH = 0;
    public static double shoot1ControlX = 112, shoot1ControlY = 75;
    public static double shoot1X = 91, shoot1Y = 80, shoot1H = -12;
    public static double drivePickup2X = 102, drivePickup2Y = 58.5, drivePickup2H = 0;
    public static double pickup2X = 133, pickup2Y = 57, pickup2H = 0;
    public static double shoot2ControlX = 102, shoot2ControlY = 63;
    public static double shoot2X = 91, shoot2Y = 80, shoot2H = -50;
    public static double drivePickup3X = 102, drivePickup3Y = 34.5, drivePickup3H = 0;
    public static double pickup3X = 133, pickup3Y = 34.5, pickup3H = 0;
    public static double shoot3ControlX = 97.62371134020621, shoot3ControlY = 34.813287514318446;
    public static double shoot3X = 84, shoot3Y = 105, shoot3H = -80;
    Pose startPose, pushBot, shoot0Control, shoot0,
            pickup1Control, pickup1, openGateControl, openGate, shoot1Control, shoot1,
            drivePickup2, pickup2, shoot2Control, shoot2,
            drivePickup3, pickup3, shoot3Control, shoot3;
    private void initializePoses(){
        startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseH));
        pushBot = new Pose(pushBotX, pushBotY, Math.toRadians(pushBotH));
        shoot0Control = new Pose(shoot0ControlX, shoot0ControlY);
        shoot0 = new Pose(shoot0X, shoot0Y, Math.toRadians(shoot0H));
        pickup1Control = new Pose(pickup1ControlX, pickup1ControlY);
        pickup1 = new Pose(pickup1X, pickup1Y, Math.toRadians(pickup1H));
        openGateControl = new Pose(openGateControlX, openGateControlY);
        openGate = new Pose(openGateX, openGateY, Math.toRadians(openGateH));
        shoot1Control = new Pose(shoot1ControlX, shoot1ControlY);
        shoot1 = new Pose(shoot1X, shoot1Y, Math.toRadians(shoot1H));
        drivePickup2 = new Pose(drivePickup2X, drivePickup2Y, Math.toRadians(drivePickup2H));
        pickup2 = new Pose(pickup2X, pickup2Y, Math.toRadians(pickup2H));
        shoot2Control = new Pose(shoot2ControlX, shoot2ControlY);
        shoot2 = new Pose(shoot2X, shoot2Y, Math.toRadians(shoot2H));
        drivePickup3 = new Pose(drivePickup3X, drivePickup3Y, Math.toRadians(drivePickup3H));
        pickup3 = new Pose(pickup3X, pickup3Y, Math.toRadians(pickup3H));
        shoot3Control = new Pose(shoot3ControlX, shoot3ControlY);
        shoot3 = new Pose(shoot3X, shoot3Y, Math.toRadians(shoot3H));
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
                .addPath(new BezierCurve(shoot0, pickup1Control, pickup1))
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
                if (startAuto){
                    follower.followPath(startPose_pushBot, true);
                    startAuto = false;
                    shootX = shoot0X;
                    shootY = shoot0Y;
                    shootH = shoot0H;
                }
                if (!follower.isBusy()){
                    follower.followPath(pushBot_shoot0, true);
                    pathState = PathState.DRIVE_SHOOT0;
                }
                break;
            case DRIVE_SHOOT0:
                if (!follower.isBusy()){
                    timeStamp = currentTime;
                    pathState = PathState.WAIT_SHOOT0;
                }
                break;
            case WAIT_SHOOT0:
                if (currentTime - timeStamp > shootTime){
                    follower.followPath(shoot0_pickup1, intakePower, false);
                    pathState = PathState.PICKUP1;
                }
                break;
            case PICKUP1:
                if (!follower.isBusy()){
                    follower.followPath(pickup1_openGate, true);
                    pathState = PathState.OPENGATE;
                    shootX = shoot1X;
                    shootY = shoot1Y;
                    shootH = shoot1H;
                }
                break;
            case DRIVE_OPENGATE:
                if (!follower.isBusy()){
                    pathState = PathState.OPENGATE;
                    timeStamp = currentTime;
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
                }
                break;
            case WAIT_SHOOT1:
                if (currentTime - timeStamp > shootTime){
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
                }
                break;
            case WAIT_SHOOT2:
                if (currentTime - timeStamp > shootTime){
                    follower.followPath(shoot2_drivePickup3, true);
                    pathState = PathState.DRIVE_PICKUP3;
                }
                break;
            case DRIVE_PICKUP3:
                if (!follower.isBusy()){
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
                }
                break;
            case WAIT_SHOOT3:
                // add line here to say "done auto'
                break;
            default:
                break;
        }
        TELE.update(); // use for debugging
    }

    // Used for changing alliance
    private double adjustXPoseBasedOnAlliance(double pose) {return (144-pose);}
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
        follower.setStartingPose(new Pose(72,72,0));
        loopTimes = new MeasuringLoopTimes();
        loopTimes.init();

        boolean initializeRobot = false;
        while (opModeInInit()){
            follower.update();

            if (gamepad1.crossWasPressed() && !initializeRobot){
                Color.redAlliance = !Color.redAlliance;

                double[] xPoses = {startPoseX, pushBotX, shoot0ControlX, shoot0X,
                        pickup1ControlX, pickup1X, openGateControlX, openGateX, shoot1ControlX, shoot1X,
                        drivePickup2X, pickup2X, shoot2ControlX, shoot2X,
                        drivePickup3X, pickup3X, shoot3ControlX, shoot3X};

                double[] headings = {startPoseH, pushBotH, shoot0H,
                        pickup1H, openGateH, shoot1H,
                        drivePickup2H, pickup2H, shoot2H,
                        drivePickup3H, pickup3H, shoot3H};

                for (int i = 0; i < xPoses.length; i++) {xPoses[i] = adjustXPoseBasedOnAlliance(xPoses[i]);}
                for (int i = 0; i < headings.length; i++) {headings[i] = adjustHeadingBasedOnAlliance(headings[i]);}
            }

            if (gamepad1.triangleWasPressed()){
                initializeRobot = true;
                initializePoses();
                follower.setPose(startPose);
                buildPaths();
//                turret.switchPipeline(Turret.PipelineMode.OBELISK);
                robot.limelight.start();
                sleep(2000);
            }

            TELE.addData("Red Alliance?", Color.redAlliance);
            TELE.addData("Initialized Robot? (Don't change this until alliance is selected)", initializeRobot);
            TELE.addData("Start Pose", follower.getPose());
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        limelightUsed = false;

        while (opModeIsActive()){
            follower.update();
            pathStateMachine();
            Pose currentPose = follower.getPose();
            teleStartPoseX = currentPose.getX();
            teleStartPoseY = currentPose.getY();
            teleStartPoseH = Math.toDegrees(currentPose.getHeading());

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            loopTimes.loop();

            TELE.addData("Avg Loop Time", loopTimes.getAvgLoopTime());
            TELE.addData("Min Loop Time", loopTimes.getMinLoopTimeOneMin());
            TELE.addData("Max Loop Time", loopTimes.getMaxLoopTimeOneMin());
            TELE.addData("X:", currentPose.getX());
            TELE.addData("Y:", currentPose.getY());
            TELE.addData("H:", currentPose.getHeading());
            TELE.update();
        }
    }
}