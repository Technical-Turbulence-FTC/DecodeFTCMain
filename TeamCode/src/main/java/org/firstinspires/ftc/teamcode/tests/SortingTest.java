package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@TeleOp
public class SortingTest extends LinearOpMode {
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

        light = Light.getInstance();

        light.init(robot.light, spindexer, turret);

        autoActions = new AutoActions(robot, drive, TELE, servos, flywheel, spindexer, targeting, targetingSettings, turret, light);

        int motif = 21;
        boolean intaking = true;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            spindexer.setIntakePower(1);
            robot.transfer.setPower(1);

            if (gamepad1.crossWasPressed()){
                motif = 21;
            } else if (gamepad1.squareWasPressed()){
                motif = 22;
            } else if (gamepad1.triangleWasPressed()){
                motif = 23;
            }
            flywheel.manageFlywheel(2500);

            if (gamepad1.leftBumperWasPressed()){
                intaking = false;
                Actions.runBlocking(
                        autoActions.prepareShootAll(
                                3,
                                5,
                                motif,
                                0.501,
                                0.501,
                                0.501
                        )
                );
            } else if (gamepad1.rightBumperWasPressed()){
                intaking = false;
                Actions.runBlocking(
                        autoActions.shootAllAuto(
                                3.5,
                                0.014,
                                0.501,
                                0.501,
                                0.501
                        )
                );
                intaking = true;
            } else if (intaking){
                spindexer.processIntake();
            }
        }
    }
}
