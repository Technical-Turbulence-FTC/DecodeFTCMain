package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.utils.Targeting.turretInterpolate;
import static org.firstinspires.ftc.teamcode.utils.Turret.turrDefault;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseH;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseY;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStartPoseX;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.Color;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;
import com.pedropathing.util.Timer;

import java.util.List;

@Config
@Autonomous (preselectTeleOp = "TeleopV3")
public class Auto12BallPedroPathing extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    Flywheel flywheel;
    Targeting targeting;
    Targeting.Settings targetingSettings;
    Follower follower;
    Turret turret;
    Spindexer spindexer;
    Servos servos;
    Timer pathTimer, opModeTimer;

    // Initialize path state machine
    enum PathState {
        DRIVE_SHOOT0,
        WAIT_SHOOT0,
        DRIVE_PICKUP1,
        PICKUP1,
        DRIVE_SHOOT1,
        WAIT_SHOOT1,
        DRIVE_PICKUP2,
        PICKUP2,
        DRIVE_SHOOT2,
        WAIT_SHOOT2,
        DRIVE_PICKUP3,
        PICKUP3,
        DRIVE_SHOOT3,
        WAIT_SHOOT3
    }
    PathState pathState = PathState.DRIVE_SHOOT0;

    // Poses
    public static double startPoseX = 110, startPoseY = 130, startPoseH = -90;
    public static double shoot0X = 88, shoot0Y = 82, shoot0H = -45;
    Pose startPose;
    Pose shoot0;

    //Building Paths
    PathChain startPose_shoot0;
    void buildPaths(){
        startPose_shoot0 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot0))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot0.getHeading())
                .build();
    }

    //Path State Machine
    void pathStateMachine(){
        switch(pathState){
            case DRIVE_SHOOT0:
                follower.followPath(startPose_shoot0);
                pathState = PathState.WAIT_SHOOT0;
                break;
            case WAIT_SHOOT0:
                boolean shootStart = false;
                if (!follower.isBusy() && !shootStart){
                    spindexer.prepareShootAllContinous();
                    shootStart = true;
                }
                if (shootStart && spindexer.shootAllComplete()){
                    //pathState = PathState.DRIVE_PICKUP1;
                    TELE.addLine("End Auto");
                    TELE.update();
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        flywheel = new Flywheel(hardwareMap);
        targeting = new Targeting();
        targetingSettings = new Targeting.Settings(0,0);
        follower = Constants.createFollower(hardwareMap);
        turret = new Turret(robot, TELE, robot.limelight);
        spindexer = new Spindexer(hardwareMap);
        opModeTimer = new Timer();
        pathTimer = new Timer();
        hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint").resetPosAndIMU();

        robot.light.setPosition(Color.LightRed);

        int allianceTicks = 0;
        while (opModeInInit()){
            boolean initalizeRobot = false;

            if (gamepad1.crossWasPressed() && !initalizeRobot){
                allianceTicks++;
                Color.redAlliance = !Color.redAlliance;
                if (Color.redAlliance){
                    robot.light.setPosition(Color.LightRed);
                } else {
                    robot.light.setPosition(Color.LightBlue);
                }
                startPoseX = 144 - startPoseX;
                startPoseH = 180 - startPoseH;
                shoot0X = 144 - shoot0X;
                shoot0H = 180 - shoot0H;

                while (startPoseH > 180) {startPoseH-=360;}
                while (startPoseH < -180) {startPoseH+=360;}

                while (shoot0H > 180) {shoot0H-=360;}
                while (shoot0H < -180) {shoot0H+=360;}
            }

            if (gamepad1.triangleWasPressed()){
                initalizeRobot = true;
            }

            if (initalizeRobot){
                follower.setStartingPose(startPose);
                buildPaths();
                sleep(2000);
                turret.setTurret(turrDefault);
                servos.setSpinPos(spinStartPos);
            }

            TELE.addData("Red Alliance?", Color.redAlliance);
            TELE.addData("Initialized Robot? (Don't change this until alliance is selected)", initalizeRobot);
            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            follower.update();
            pathStateMachine();
            Pose currentPose = follower.getPose();
            teleStartPoseX = currentPose.getX();
            teleStartPoseY = currentPose.getY();
            teleStartPoseH = Math.toDegrees(currentPose.getHeading());
            turret.trackGoal(currentPose);
            targetingSettings = targeting.calculateSettings(currentPose.getX(), currentPose.getY(), currentPose.getHeading(), 0, turretInterpolate);

            double voltage = robot.voltage.getVoltage();
            flywheel.setPIDF(Robot.shooterPIDF_P, Robot.shooterPIDF_I, Robot.shooterPIDF_D, Robot.shooterPIDF_F / voltage);
            flywheel.manageFlywheel(targetingSettings.flywheelRPM);
            servos.setHoodPos(targetingSettings.hoodAngle);
        }
    }
}
