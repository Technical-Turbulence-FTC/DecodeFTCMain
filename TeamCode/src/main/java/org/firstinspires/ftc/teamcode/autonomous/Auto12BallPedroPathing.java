package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.utils.Targeting.turretInterpolate;
import static org.firstinspires.ftc.teamcode.utils.Turret.limelightUsed;
import static org.firstinspires.ftc.teamcode.utils.Turret.turrDefault;
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
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utilsv2.Turret;

import java.util.List;

@Config
@Autonomous (preselectTeleOp = "TeleopV4")
public class Auto12BallPedroPathing extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
//    Flywheel flywheel;
//    Targeting targeting;
//    Targeting.Settings targetingSettings;
    Follower follower;
//    Turret turret;
//    Spindexer spindexer;
//    Servos servos;
    MeasuringLoopTimes loopTimes;

    // Wait Times
    public static double shootTime = 2;

    // Extra Variables
    public static double intakePower = 0.3;
    double shootX, shootY, shootH;

    // Initialize path state machine
    private enum PathState {
        DRIVE_SHOOT0, WAIT_SHOOT0,
        DRIVE_PICKUP1, PICKUP1, DRIVE_SHOOT1, WAIT_SHOOT1,
        DRIVE_PICKUP2, PICKUP2, DRIVE_SHOOT2, WAIT_SHOOT2,
        DRIVE_PICKUP3, PICKUP3, DRIVE_SHOOT3, WAIT_SHOOT3
    }
    PathState pathState = PathState.DRIVE_SHOOT0;

    // Poses
    public static double startPoseX = 112, startPoseY = 132.5, startPoseH = -90;
    public static double shoot0X = 106, shoot0Y = 106, shoot0H = -40;
    public static double drivePickup1X = 102, drivePickup1Y = 82, drivePickup1H = 0;
    public static double pickup1X = 126, pickup1Y = 82, pickup1H = 0;
    public static double shoot1X = 86, shoot1Y = 82, shoot1H = -80;
    public static double drivePickup2ControlX = 91.69828844730904, drivePickup2ControlY = 66.724457099909;
    public static double drivePickup2X = 102, drivePickup2Y = 58.5, drivePickup2H = 0;
    public static double pickup2X = 132, pickup2Y = 57, pickup2H = 0;
    public static double shoot2ControlX = 86, shoot2ControlY = 57;
    public static double shoot2X = 86, shoot2Y = 82, shoot2H = -90;
    public static double drivePickup3ControlX = 97.97800291788306, drivePickup3ControlY = 50.10765863138859;
    public static double drivePickup3X = 102, drivePickup3Y = 34.5, drivePickup3H = 0;
    public static double pickup3X = 132, pickup3Y = 34.5, pickup3H = 0;
    public static double shoot3ControlX = 86, shoot3ControlY = 34.5;
    public static double shoot3X = 84, shoot3Y = 102, shoot3H = -90;
    Pose startPose, shoot0,
            drivePickup1, pickup1, shoot1,
            drivePickup2Control, drivePickup2, pickup2, shoot2Control, shoot2,
            drivePickup3Control, drivePickup3, pickup3, shoot3Control, shoot3;
    private void initializePoses(){
        startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseH));
        shoot0 = new Pose(shoot0X, shoot0Y, Math.toRadians(shoot0H));
        drivePickup1 = new Pose(drivePickup1X, drivePickup1Y, Math.toRadians(drivePickup1H));
        pickup1 = new Pose(pickup1X, pickup1Y, Math.toRadians(pickup1H));
        shoot1 = new Pose(shoot1X, shoot1Y, Math.toRadians(shoot1H));
        drivePickup2Control = new Pose(drivePickup2ControlX, drivePickup2ControlY);
        drivePickup2 = new Pose(drivePickup2X, drivePickup2Y, Math.toRadians(drivePickup2H));
        pickup2 = new Pose(pickup2X, pickup2Y, Math.toRadians(pickup2H));
        shoot2Control = new Pose(shoot2ControlX, shoot2ControlY);
        shoot2 = new Pose(shoot2X, shoot2Y, Math.toRadians(shoot2H));
        drivePickup3Control = new Pose(drivePickup3ControlX, drivePickup3ControlY);
        drivePickup3 = new Pose(drivePickup3X, drivePickup3Y, Math.toRadians(drivePickup3H));
        pickup3 = new Pose(pickup3X, pickup3Y, Math.toRadians(pickup3H));
        shoot3Control = new Pose(shoot3ControlX, shoot3ControlY);
        shoot3 = new Pose(shoot3X, shoot3Y, Math.toRadians(shoot3H));
    } // add poses to void

    //Building Paths
    PathChain startPose_shoot0,
            shoot0_drivePickup1, drivePickup1_pickup1, pickup1_shoot1,
            shoot1_drivePickup2, drivePickup2_pickup2, pickup2_shoot2,
            shoot2_drivePickup3, drivePickup3_pickup3, pickup3_shoot3;
    private void buildPaths(){
        startPose_shoot0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot0))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot0.getHeading())
                .build();

        shoot0_drivePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(shoot0, drivePickup1))
                .setLinearHeadingInterpolation(shoot0.getHeading(), drivePickup1.getHeading())
                .build();

        drivePickup1_pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(drivePickup1, pickup1))
                .setTangentHeadingInterpolation()
                .build();

        pickup1_shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, shoot1))
                .setLinearHeadingInterpolation(pickup1.getHeading(), shoot1.getHeading())
                .build();

        shoot1_drivePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot1, drivePickup2Control, drivePickup2))
                .setLinearHeadingInterpolation(shoot1.getHeading(), drivePickup2.getHeading())
                .build();

        drivePickup2_pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(drivePickup2, pickup2))
                .setConstantHeadingInterpolation(pickup2.getHeading())
                .build();

        pickup2_shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2, shoot2Control, shoot2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), shoot2.getHeading())
                .build();

        shoot2_drivePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot2, drivePickup3Control, drivePickup3))
                .setLinearHeadingInterpolation(shoot2.getHeading(), drivePickup3.getHeading())
                .build();

        drivePickup3_pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(drivePickup3, pickup3))
                .setTangentHeadingInterpolation()
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
            case DRIVE_SHOOT0:
                if (startAuto){
                    follower.followPath(startPose_shoot0, true);
                    startAuto = false;
                    shootX = shoot0X;
                    shootY = shoot0Y;
                    shootH = shoot0H;
                }
                driveShoot(PathState.WAIT_SHOOT0, currentTime);
                break;
            case WAIT_SHOOT0:
                waitShoot(PathState.DRIVE_PICKUP1, shoot0_drivePickup1, currentTime);
                break;
            case DRIVE_PICKUP1:
                drivePickup(PathState.PICKUP1, drivePickup1_pickup1);
                break;
            case PICKUP1:
                pickup(PathState.DRIVE_SHOOT1, pickup1_shoot1);
                shootX = shoot1X;
                shootY = shoot1Y;
                shootH = shoot1H;
                break;
            case DRIVE_SHOOT1:
                intakePowerDown(timeStamp, currentTime);
                driveShoot(PathState.WAIT_SHOOT1, currentTime);
                break;
            case WAIT_SHOOT1:
                waitShoot(PathState.DRIVE_PICKUP2, shoot1_drivePickup2, currentTime);
                break;
            case DRIVE_PICKUP2:
                drivePickup(PathState.PICKUP2, drivePickup2_pickup2);
                break;
            case PICKUP2:
                pickup(PathState.DRIVE_SHOOT2, pickup2_shoot2);
                shootX = shoot2X;
                shootY = shoot2Y;
                shootH = shoot2H;
                break;
            case DRIVE_SHOOT2:
                intakePowerDown(timeStamp, currentTime);
                driveShoot(PathState.WAIT_SHOOT2, currentTime);
                break;
            case WAIT_SHOOT2:
                waitShoot(PathState.DRIVE_PICKUP3, shoot2_drivePickup3, currentTime);
                break;
            case DRIVE_PICKUP3:
                drivePickup(PathState.PICKUP3, drivePickup3_pickup3);
                break;
            case PICKUP3:
                pickup(PathState.DRIVE_SHOOT3, pickup3_shoot3);
                shootX = shoot3X;
                shootY = shoot3Y;
                shootH = shoot3H;
                break;
            case DRIVE_SHOOT3:
                intakePowerDown(timeStamp, currentTime);
                driveShoot(PathState.WAIT_SHOOT3, currentTime);
                break;
            case WAIT_SHOOT3:
//                if (spindexer.shootAllComplete()){
//                    spindexer.resetSpindexer();
//                    TELE.addLine("Done Auto");
//                }
                break;
            default:
                break;
        }
        TELE.update(); // use for debugging
    }

    // Voids for State Machine
    private void intakePowerDown(double stamp, double currentTime) {
        double pow = (stamp - currentTime) / 2; // adjust denominator to see how much time to adjust
        if (pow < -1) {pow = 0;}
//        spindexer.setIntakePower(pow);
    }
    private void driveShoot(PathState nextState, double currentTime){
        if (!follower.isBusy()){
            pathState = nextState;
            timeStamp = currentTime;
//            spindexer.prepareShootAllContinous();
        }
    }
    private void waitShoot(PathState nextState, PathChain nextPath, double currentTime) {
        if (currentTime - timeStamp > shootTime) { // spindexer.shootAllComplete() ||
//            spindexer.resetSpindexer();
            pathState = nextState;
            follower.followPath(nextPath, true);
//            spindexer.setIntakePower(1);
        }
    }
    private void drivePickup(PathState nextState, PathChain nextPath) {
        if (!follower.isBusy()) {
            pathState = nextState;
            follower.followPath(nextPath, intakePower, false);
        }
    }
    private void pickup(PathState nextState, PathChain nextPath) {
        if (!follower.isBusy()) {
            pathState = nextState;
            follower.followPath(nextPath, true);
        }
    }

    // Helps manage spindexer
    private boolean driveToShoot(){
        return pathState == PathState.DRIVE_SHOOT0 ||
                pathState == PathState.DRIVE_SHOOT1 ||
                pathState == PathState.DRIVE_SHOOT2 ||
                pathState == PathState.DRIVE_SHOOT3;
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
//        flywheel = new Flywheel(hardwareMap);
//        targeting = new Targeting();
//        targetingSettings = new Targeting.Settings(0,0);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,0));
//        turret = new Turret(robot, TELE, robot.limelight);
//        spindexer = new Spindexer(hardwareMap);
//        servos = new Servos(hardwareMap);
        loopTimes = new MeasuringLoopTimes();
        loopTimes.init();

//        robot.light.setPosition(Color.LightRed);

        boolean initializeRobot = false;
        while (opModeInInit()){
            follower.update();

            if (gamepad1.crossWasPressed() && !initializeRobot){
                Color.redAlliance = !Color.redAlliance;
//                if (Color.redAlliance){
//                    robot.light.setPosition(Color.LightRed);
//                } else {
//                    robot.light.setPosition(Color.LightBlue);
//                }

                double[] xPoses = {startPoseX, shoot0X,
                        drivePickup1X, pickup1X, shoot1X,
                        drivePickup2ControlX, drivePickup2X, pickup2X, shoot2ControlX, shoot2X,
                        drivePickup3ControlX, drivePickup3X, pickup3X, shoot3ControlX, shoot3X};

                double[] headings = {startPoseH, shoot0H,
                        drivePickup1H, pickup1H, shoot1H,
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
                sleep(2000);

//                turret.setTurret(turrDefault);
//                servos.setSpinPos(spinStartPos);
            }

            TELE.addData("Red Alliance?", Color.redAlliance);
            TELE.addData("Initialized Robot? (Don't change this until alliance is selected)", initializeRobot);
            TELE.addData("Start Pose", follower.getPose());
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

//        robot.transfer.setPower(1);
        limelightUsed = false;

        while (opModeIsActive()){
            follower.update();
            pathStateMachine();
            Pose currentPose = follower.getPose();
//            teleStartPoseX = currentPose.getX();
//            teleStartPoseY = currentPose.getY();
//            teleStartPoseH = Math.toDegrees(currentPose.getHeading());
//
//            turret.trackGoal(new Pose(shootX, shootY, Math.toRadians(shootH)));
//            targetingSettings = targeting.calculateSettings(shootX, shootY, Math.toRadians(shootH), 0, turretInterpolate);
//
//            double voltage = robot.voltage.getVoltage();
//            flywheel.setPIDF(Robot.shooterPIDF_P, Robot.shooterPIDF_I, Robot.shooterPIDF_D, Robot.shooterPIDF_F / voltage);
//            flywheel.manageFlywheel(targetingSettings.flywheelRPM);
//            servos.setHoodPos(targetingSettings.hoodAngle);
//
//            if (driveToShoot()){servos.setSpinPos(spinStartPos);}
//            else {spindexer.processIntake();}

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