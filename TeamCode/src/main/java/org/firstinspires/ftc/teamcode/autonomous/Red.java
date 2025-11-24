package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Poses.*;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constants.ServoPositions;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utils.Robot;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;



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








        TrajectoryActionBuilder traj1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1 );


        TrajectoryActionBuilder traj2 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .turnTo(Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(x2, y2), h2 );


        TrajectoryActionBuilder traj3 = drive.actionBuilder(new Pose2d(x2, y2, h2))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1 );

        TrajectoryActionBuilder traj4 = drive.actionBuilder(new Pose2d(x1, y1, h1))

                .strafeToLinearHeading(new Vector2d(x2_b, y2_b), h2_b )

                .strafeToLinearHeading(new Vector2d(x3, y3), h3 );


        TrajectoryActionBuilder traj5 = drive.actionBuilder(new Pose2d(x3, y3, h3))
                .strafeToLinearHeading(new Vector2d(x1, y1), h1 );


        TrajectoryActionBuilder traj6 = drive.actionBuilder(new Pose2d(x1, y1, h1))
                .strafeToLinearHeading(new Vector2d(x1, y1+30), h1 );

        while(opModeInInit()) {

            if (gamepad2.dpadUpWasPressed()){
                hoodDefault -= 0.02;
            }

            if (gamepad2.dpadDownWasPressed()){
                hoodDefault += 0.02;
            }

            robot.hood.setPosition(hoodDefault);

            shooter.setTurretPosition(0.33);

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
                            traj1.build()
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
                    traj2.build()
            );


            Actions.runBlocking(
                    traj3.build()
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
                    traj4.build()
            );


            Actions.runBlocking(
                    traj5.build()
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
                    traj6.build()
            );

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            TELE.addLine("finished");

            TELE.update();

            sleep (2000);


        }

    }
}
