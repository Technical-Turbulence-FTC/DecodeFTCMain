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
            TELE.addData("spindexer", 1.111111111*((robot.spin1Pos.getVoltage() / 3.3) - 0.05));
            TELE.addData("hood", 1.111111111*((robot.hoodPos.getVoltage() / 3.3) - 0.05));
            TELE.addData("transferServo", 1.111111111*((robot.transferServoPos.getVoltage() / 3.3) - 0.05));
            TELE.addData("turret", 1.111111111*((robot.turr1Pos.getVoltage() / 3.3) - 0.05));
            TELE.update();
        }
    }
}
