package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.tests.PIDServoTest.*;

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
    Servos servo;

    public static double spindexPos = 0.501;
    public static double turretPos = 0.501;
    public static double transferPos = 0.501;
    public static double hoodPos = 0.501;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = new Servos(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            if (spindexPos != 0.501 && !servo.spinEqual(spindexPos)){
                double pos = servo.setSpinPos(spindexPos);
                robot.spin1.setPower(pos);
                robot.spin2.setPower(-pos);
            } else{
                robot.spin1.setPower(0);
                robot.spin2.setPower(0);
            }
            if (turretPos != 0.501 && !servo.turretEqual(turretPos)){
                double pos = servo.setTurrPos(turretPos);
                robot.turr1.setPower(pos);
                robot.turr2.setPower(-pos);
            } else {
                robot.turr1.setPower(0);
                robot.turr2.setPower(0);
            }
            if (transferPos != 0.501){
                robot.transferServo.setPosition(transferPos);
            }
            if (hoodPos != 0.501){
                robot.hood.setPosition(hoodPos);
            }
            TELE.addData("spindexer", servo.getSpinPos());
            TELE.addData("hood", 1-scalar*((robot.hoodPos.getVoltage() - restPos) / 3.3));
            TELE.addData("transferServo", scalar*((robot.transferServoPos.getVoltage() - restPos) / 3.3));
            TELE.addData("turret", servo.getTurrPos());
            TELE.addData("spindexer voltage", robot.spin1Pos.getVoltage());
            TELE.addData("hood voltage", robot.hoodPos.getVoltage());
            TELE.addData("transferServo voltage", robot.transferServoPos.getVoltage());
            TELE.addData("turret voltage", robot.turr1Pos.getVoltage());
            TELE.addData("Spin Equal", servo.spinEqual(spindexPos));
            TELE.update();
        }
    }
}
