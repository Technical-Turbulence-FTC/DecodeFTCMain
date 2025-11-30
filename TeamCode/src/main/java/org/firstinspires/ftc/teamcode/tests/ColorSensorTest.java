package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp
@Config
public class ColorSensorTest extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

// ----- COLOR 1 -----
            double green1 = robot.color1.getNormalizedColors().green;
            double blue1  = robot.color1.getNormalizedColors().blue;
            double red1   = robot.color1.getNormalizedColors().red;

            TELE.addData("Color1 toColor", robot.color1.getNormalizedColors().toColor());
            TELE.addData("Color1 green", green1 / (green1 + blue1 + red1));
            TELE.addData("Color1 distance (mm)", robot.color1.getDistance(DistanceUnit.MM));


// ----- COLOR 2 -----
            double green2 = robot.color2.getNormalizedColors().green;
            double blue2  = robot.color2.getNormalizedColors().blue;
            double red2   = robot.color2.getNormalizedColors().red;

            TELE.addData("Color2 toColor", robot.color2.getNormalizedColors().toColor());
            TELE.addData("Color2 green", green2 / (green2 + blue2 + red2));
            TELE.addData("Color2 distance (mm)", robot.color2.getDistance(DistanceUnit.MM));


// ----- COLOR 3 -----
            double green3 = robot.color3.getNormalizedColors().green;
            double blue3  = robot.color3.getNormalizedColors().blue;
            double red3   = robot.color3.getNormalizedColors().red;

            TELE.addData("Color3 toColor", robot.color3.getNormalizedColors().toColor());
            TELE.addData("Color3 green", green3 / (green3 + blue3 + red3));
            TELE.addData("Color3 distance (mm)", robot.color3.getDistance(DistanceUnit.MM));


            TELE.update();
        }
    }


}
