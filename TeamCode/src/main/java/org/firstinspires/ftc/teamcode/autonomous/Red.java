package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@Autonomous
public class Red extends LinearOpMode {

    Robot robot;

    MultipleTelemetry TELE;

    MecanumDrive drive;

    AprilTag aprilTag;

    Shooter shooter;



    public static double angle = 0.1;

    Spindexer spindexer;

    Transfer transfer;





    @Override
    public void runOpMode() throws InterruptedException {




        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        drive = new MecanumDrive(hardwareMap, new Pose2d(
                0,0,0
        ));

        aprilTag = new AprilTag(robot, TELE);

        shooter = new Shooter(robot, TELE);

        spindexer = new Spindexer(robot, TELE);

        spindexer.outtake3();

        shooter.setShooterMode("MANUAL");

        shooter.sethoodPosition(hoodDefault);

        transfer = new Transfer(robot);

        transfer.setTransferPosition(transferServo_out);

        Intake intake = new Intake(robot);

        robot.hood.setPosition(hoodDefault);








        TrajectoryActionBuilder shoot0 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1 );


        TrajectoryActionBuilder pickup1 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .turnTo(Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(x2b, y2b), h2b);


        TrajectoryActionBuilder shoot1 = drive.actionBuilder(new Pose2d(x2b, y2b, h2b))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1 );

        TrajectoryActionBuilder pickup2 = drive.actionBuilder(new Pose2d(x1, y1, h1))

                .strafeToLinearHeading(new Vector2d(x3a, y3a), h3a)

                .strafeToLinearHeading(new Vector2d(x3b, y3b), h3b);


        TrajectoryActionBuilder shoot2 = drive.actionBuilder(new Pose2d(x3b, y3b, h3b))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1 );


        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .strafeToLinearHeading(new Vector2d(x1, y1+30), h1 );

        while(opModeInInit()) {

            if (gamepad2.dpadUpWasPressed()){
                hoodDefault -= 0.01;
            }

            if (gamepad2.dpadDownWasPressed()){
                hoodDefault += 0.01;
            }

            robot.hood.setPosition(hoodDefault);

            shooter.setTurretPosition(turret_red);

            aprilTag.initTelemetry();

            aprilTag.update();
            shooter.update();

            TELE.update();
        }


        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()){

            robot.hood.setPosition(hoodDefault);

            transfer.setTransferPower(1);

            transfer.update();




            Actions.runBlocking(
                    new ParallelAction(
                            shoot0.build()
                    )
            );


            transfer.setTransferPower(1);

            transfer.update();

            shooter.setManualPower(1);

            double stamp = getRuntime();


            stamp = getRuntime();

            while (getRuntime()-stamp<4.5) {



                shooter.moveTurret(0.31);

                double time = getRuntime()-stamp;


                if (time < 0.3) {


                    transfer.transferOut();
                    transfer.setTransferPower(1);
                } else if (time < 1) {

                    spindexer.outtake3();


                } else if (time < 1.4) {


                    transfer.transferIn();
                } else if (time < 2.6) {
                    transfer.transferOut();

                    spindexer.outtake2();



                } else if (time < 3.0) {


                    transfer.transferIn();
                } else if (time < 4.0) {

                    transfer.transferOut();

                    spindexer.outtake1();

                } else if (time < 4.4) {
                    transfer.transferIn();
                } else {

                    transfer.transferOut();
                    spindexer.outtake3();

                    shooter.setManualPower(0);

                }

                transfer.update();

                shooter.update();

                spindexer.update();

            }

            spindexer.outtake3();

            robot.intake.setPower(1);

            Actions.runBlocking(
                    pickup1.build()
            );


            Actions.runBlocking(
                    shoot1.build()
            );

            shooter.setManualPower(1);




            robot.intake.setPower(0);

            spindexer.outtake3();


            stamp = getRuntime();

            shooter.setManualPower(1);
            while (getRuntime()-stamp<4.5) {



                shooter.moveTurret(0.31);

                double time = getRuntime()-stamp;


                if (time < 0.3) {


                    transfer.transferOut();
                    transfer.setTransferPower(1);
                } else if (time < 1) {

                    spindexer.outtake3();


                } else if (time < 1.4) {


                    transfer.transferIn();
                } else if (time < 2.6) {
                    transfer.transferOut();

                    spindexer.outtake2();



                } else if (time < 3.0) {


                    transfer.transferIn();
                } else if (time < 4.0) {

                    transfer.transferOut();

                    spindexer.outtake1();

                } else if (time < 4.4) {
                    transfer.transferIn();
                } else {

                    transfer.transferOut();
                    spindexer.outtake3();

                    shooter.setManualPower(0);

                }

                transfer.update();

                shooter.update();

                spindexer.update();

            }

            spindexer.outtake3();
            robot.intake.setPower(1);

            Actions.runBlocking(
                    pickup2.build()
            );


            Actions.runBlocking(
                    shoot2.build()
            );

            shooter.setManualPower(1);




            robot.intake.setPower(0);


            stamp = getRuntime();

            shooter.setManualPower(1);

            while (getRuntime()-stamp<4.5) {



                shooter.moveTurret(0.31);

                double time = getRuntime()-stamp;


                if (time < 0.3) {


                    transfer.transferOut();
                    transfer.setTransferPower(1);
                } else if (time < 1) {

                    spindexer.outtake3();


                } else if (time < 1.4) {


                    transfer.transferIn();
                } else if (time < 2.6) {
                    transfer.transferOut();

                    spindexer.outtake2();



                } else if (time < 3.0) {


                    transfer.transferIn();
                } else if (time < 4.0) {

                    transfer.transferOut();

                    spindexer.outtake1();

                } else if (time < 4.4) {
                    transfer.transferIn();
                } else {

                    transfer.transferOut();
                    spindexer.outtake3();

                    shooter.setManualPower(0);

                }

                transfer.update();

                shooter.update();

                spindexer.update();

            }

            spindexer.outtake3();

            Actions.runBlocking(
                    park.build()
            );

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            TELE.addLine("finished");

            TELE.update();

            sleep (2000);


        }

    }
}
