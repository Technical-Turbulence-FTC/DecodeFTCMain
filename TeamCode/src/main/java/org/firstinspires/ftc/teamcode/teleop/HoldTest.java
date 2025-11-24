package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.Poses.h1;
import static org.firstinspires.ftc.teamcode.constants.Poses.x1;
import static org.firstinspires.ftc.teamcode.constants.Poses.y1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Robot;


@TeleOp
@Config
public class HoldTest extends LinearOpMode {

    Robot robot;
    MecanumDrive drive;

    public TranslationalVelConstraint VEL_CONSTRAINT = new TranslationalVelConstraint(200);
    public ProfileAccelConstraint ACCEL_CONSTRAINT = new ProfileAccelConstraint(-Math.abs(60), 200);


    @Override
    public void runOpMode() throws InterruptedException {

        MultipleTelemetry TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        robot = new Robot(hardwareMap);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));


        TrajectoryActionBuilder traj1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(0.01, 0.01), 0, VEL_CONSTRAINT, ACCEL_CONSTRAINT);

        waitForStart();


        while (opModeIsActive()){

            drive.updatePoseEstimate();

            Pose2d currentPos = drive.localizer.getPose();



            TrajectoryActionBuilder traj2 = drive.actionBuilder(currentPos)
                    .strafeToLinearHeading(new Vector2d(0, 0), 0, VEL_CONSTRAINT, ACCEL_CONSTRAINT);

            if (Math.abs(currentPos.position.x) >0.5 || Math.abs(currentPos.position.y)>0.5) {
                Actions.runBlocking(
                        traj2.build()
                );
            }





        }


    }
}
