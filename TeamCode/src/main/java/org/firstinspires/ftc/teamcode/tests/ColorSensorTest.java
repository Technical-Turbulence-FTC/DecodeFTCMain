package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp
@Config
public class ColorSensorTest extends LinearOpMode{
    Robot robot;
    MultipleTelemetry TELE;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            TELE.addData("Green1:", robot.pin1.getState());
            TELE.addData("Purple1:", robot.pin0.getState());
            TELE.addData("Green2:", robot.pin3.getState());
            TELE.addData("Purple2:", robot.pin2.getState());
            TELE.addData("Green3:", robot.pin5.getState());
            TELE.addData("Purple3:", robot.pin4.getState());
            TELE.addData("Hello Keshav (analog)", robot.analogInput.getVoltage());
            TELE.addData("Hello again (analog2)", robot.analogInput2.getVoltage());

            TELE.update();
        }
    }

}
