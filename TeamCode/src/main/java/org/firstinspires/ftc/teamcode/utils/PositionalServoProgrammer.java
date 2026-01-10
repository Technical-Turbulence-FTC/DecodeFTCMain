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
    public static double spindexPow = 0.0;
    public static double spinHoldPow = 0.0;
    public static double turretPos = 0.501;
    public static double turretPow = 0.0;
    public static double turrHoldPow = 0.0;
    public static double transferPos = 0.501;
    public static double hoodPos = 0.501;
    public static int mode = 0; //0 for positional, 1 for power

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = new Servos(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            if (spindexPos != 0.501 && !servo.spinEqual(spindexPos) && mode == 0){
                double pos = servo.setSpinPos(spindexPos);
                robot.spin1.setPower(pos);
                robot.spin2.setPower(-pos);
            } else if (mode == 0){
                robot.spin1.setPower(spinHoldPow);
                robot.spin2.setPower(spinHoldPow);
            } else {
                robot.spin1.setPower(spindexPow);
                robot.spin2.setPower(-spindexPow);
            }
            if (turretPos != 0.501 && !servo.turretEqual(turretPos)){
                double pos = servo.setTurrPos(turretPos);
                robot.turr1.setPower(pos);
                robot.turr2.setPower(-pos);
            } else if (mode == 0){
                robot.turr1.setPower(turrHoldPow);
                robot.turr2.setPower(turrHoldPow);
            } else {
                robot.turr1.setPower(turretPow);
                robot.turr2.setPower(-turretPow);
            }
            if (transferPos != 0.501){
                robot.transferServo.setPosition(transferPos);
            }
            if (hoodPos != 0.501){
                robot.hood.setPosition(hoodPos);
            }
            TELE.addData("spindexer", servo.getSpinPos());
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
