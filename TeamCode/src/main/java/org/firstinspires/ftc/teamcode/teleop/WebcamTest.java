package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;


@TeleOp
@Config
public class WebcamTest extends LinearOpMode {

    AprilTag webcam;

    MultipleTelemetry TELE;

    Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        webcam = new AprilTag(robot, TELE);

        webcam.turnTelemetryOn(true);




        while(opModeInInit()){

            webcam.initTelemetry();

            TELE.update();

        };

        if(isStopRequested()) return;


        while (opModeIsActive()){

            webcam.update();

            TELE.update();

        }


    }
}
