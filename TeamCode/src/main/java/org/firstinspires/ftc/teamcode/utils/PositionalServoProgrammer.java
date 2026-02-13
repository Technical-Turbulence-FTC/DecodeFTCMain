package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.tests.PIDServoTest.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
@Config
public class PositionalServoProgrammer extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    Servos servo;

    public static double spindexPos = 0.501;
    public static double turretPos = 0.501;
    public static double transferPos = 0.501;
    public static double hoodPos = 0.501;
    public static double light = 0.501;

    Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = new Servos(hardwareMap);

        turret = new Turret(robot, TELE, robot.limelight );
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            if (spindexPos != 0.501){
                robot.spin1.setPosition(spindexPos);
                robot.spin2.setPosition(1-spindexPos);
            }
            if (turretPos != 0.501){
                robot.turr1.setPosition(turretPos);
                robot.turr2.setPosition(1-turretPos);
            }
            if (transferPos != 0.501){
                robot.transferServo.setPosition(transferPos);
            }
            if (hoodPos != 0.501){
                robot.hood.setPosition(hoodPos);
            }
            if (light !=0.501){
                robot.light.setPosition(light);
            }

            TELE.addData("spindexer pos", servo.getSpinPos());
            TELE.addData("turret pos", robot.turr1.getPosition());
            TELE.addData("spindexer voltage 1", robot.spin1Pos.getVoltage());
            TELE.addData("spindexer voltage 2", robot.spin2Pos.getVoltage());
            TELE.addData("hood pos", robot.hood.getPosition());
            TELE.addData("transferServo voltage", robot.transferServoPos.getVoltage());
            TELE.addData("tpos ", turret.getTurrPos() );
            TELE.update();
        }
    }
}
