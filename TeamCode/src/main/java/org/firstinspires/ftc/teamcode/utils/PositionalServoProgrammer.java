package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class PositionalServoProgrammer extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    public static double spindexPos = 0.501;
    public static double turretPos = 0.501;
    public static double transferPos = 0.501;
    public static double hoodPos = 0.501;

    public static double scalar = 1.112;
    public static double restPos = 0.158;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
            TELE.addData("spindexer", scalar*((robot.spin1Pos.getVoltage() - restPos) / 3.3));
            TELE.addData("hood", 1-scalar*((robot.hoodPos.getVoltage() - restPos) / 3.3));
            TELE.addData("transferServo", scalar*((robot.transferServoPos.getVoltage() - restPos) / 3.3));
            TELE.addData("turret", scalar*((robot.turr1Pos.getVoltage() - restPos) / 3.3));
            TELE.addData("spindexerA", robot.spin1Pos.getVoltage());
            TELE.addData("hoodA", robot.hoodPos.getVoltage());
            TELE.addData("transferServoA", robot.transferServoPos.getVoltage());
            TELE.addData("turretA", robot.turr1Pos.getVoltage());
            TELE.update();
        }
    }
}
