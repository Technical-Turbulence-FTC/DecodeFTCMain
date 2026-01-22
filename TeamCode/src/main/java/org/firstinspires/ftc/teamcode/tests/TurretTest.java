package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.Poses.goalPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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

        AprilTagWebcam webcam = new AprilTagWebcam();
        webcam.init(robot, TELE);

        Turret turret = new Turret(robot, TELE, webcam);
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(10, 0,0));

        while(opModeIsActive()){

            drive.updatePoseEstimate();
            Pose2d robotPose = drive.localizer.getPose();
            
            double dx = goalPose.position.x - robotPose.position.x;
            double dy = goalPose.position.y - robotPose.position.y;

            double heading = robotPose.heading.toDouble();

            // field vector -> robot frame... avoids double calculation
            double relX =  dx * Math.cos(-heading) - dy * Math.sin(-heading);
            double relY =  dx * Math.sin(-heading) + dy * Math.cos(-heading);

            Pose2d deltaPos = new Pose2d(
                    new Vector2d(relX, relY),
                    robotPose.heading
            );

            
            turret.trackGoal(deltaPos);
            
            TELE.addData("Robot Pose", robotPose);
            TELE.addData("Goal Pose", goalPose);
            TELE.addData("Delta Pos", deltaPos);
            TELE.update();
        }

    }
}
