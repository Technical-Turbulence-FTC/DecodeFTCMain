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
            if (spindexPos != 0.501 && !servo.spinEqual(spindexPos)){
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
            // To check configuration of spindexer:
            // Set "mode" to 1 and spindexPow to 0.1
            // If the spindexer is turning clockwise, the servos are reversed. Swap the configuration of the two servos, DO NOT TOUCH THE ACTUAL CODE
            // Do the previous test again to confirm
            // Set "mode" to 0 but keep spindexPos at 0.501
            // Manually turn the spindexer until "spindexer pos" is set close to 0
            // Check each spindexer voltage:
                // If "spindexer voltage 1" is closer to 0 than "spindexer voltage 2," then you are done!
                // If "spindexer voltage 2" is closer to 0 than "spindexer voltage 1," swap the two spindexer analog inputs in the configuration, DO NOT TOUCH THE ACTUAL CODE
            //TODO: @KeshavAnandCode do the above please

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
