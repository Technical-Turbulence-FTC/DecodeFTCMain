package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Turret;

@TeleOp
@Config
public class TurretTest extends LinearOpMode {
    public static boolean zeroTurr = false;
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

            TELE.addData("tpos", turret.getTurrPos());
            TELE.addData("Limelight tx", turret.getBearing());
            TELE.addData("Limelight ty", turret.getTy());
            TELE.addData("Limelight X", turret.getLimelightX());
            TELE.addData("Limelight Y", turret.getLimelightY());

            if(zeroTurr){
                turret.zeroTurretEncoder();
            }

            TELE.update();
        }

    }
}
