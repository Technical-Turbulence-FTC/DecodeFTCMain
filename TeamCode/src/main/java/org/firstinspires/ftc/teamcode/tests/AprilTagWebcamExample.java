package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@TeleOp
public class AprilTagWebcamExample extends OpMode {

    MultipleTelemetry TELE;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        aprilTagWebcam.init(new Robot(hardwareMap), TELE);

    }

    @Override
    public void loop() {

        aprilTagWebcam.update();
        aprilTagWebcam.displayAllTelemetry();
        TELE.update();

    }
}
