package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
@Config
public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap);
        MultipleTelemetry TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        Turret turret = new Turret(robot, TELE, robot.limelight);
        waitForStart();


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(15, 0,0));

        while(opModeIsActive()){

            drive.updatePoseEstimate();
            turret.trackGoal(drive.localizer.getPose());


            TELE.update();
        }

    }
}
