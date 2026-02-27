package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.Color.colorFilterAlpha;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@TeleOp
public class ColorTest extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double color1Distance = 0;
        double color2Distance = 0;
        double color3Distance = 0;

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){


            NormalizedRGBA color1RGBA = robot.color1.getNormalizedColors();

            double gP1 = color1RGBA.green / (color1RGBA.green + color1RGBA.red + color1RGBA.blue);

            double dist1 = robot.color1.getDistance(DistanceUnit.MM);
            color1Distance = (colorFilterAlpha * dist1) + ((1-colorFilterAlpha) * color1Distance);

            TELE.addData("Color1 toColor", robot.color1.getNormalizedColors().toColor());
            TELE.addData("Color1 green", gP1);
            TELE.addData("Color1 distance (mm)", color1Distance);


// ----- COLOR 2 -----
            double green2 = robot.color2.getNormalizedColors().green;
            double blue2 = robot.color2.getNormalizedColors().blue;
            double red2 = robot.color2.getNormalizedColors().red;
            double dist2 = robot.color2.getDistance(DistanceUnit.MM);
            color2Distance = (colorFilterAlpha * dist2) + ((1-colorFilterAlpha) * color2Distance);

            TELE.addData("Color2 toColor", robot.color2.getNormalizedColors().toColor());
            TELE.addData("Color2 green", green2 / (green2 + blue2 + red2));
            TELE.addData("Color2 distance (mm)", color2Distance);

// ----- COLOR 3 -----
            double green3 = robot.color3.getNormalizedColors().green;
            double blue3 = robot.color3.getNormalizedColors().blue;
            double red3 = robot.color3.getNormalizedColors().red;
            double dist3 = robot.color3.getDistance(DistanceUnit.MM);
            color3Distance = (colorFilterAlpha * dist3) + ((1-colorFilterAlpha) * color3Distance);

            TELE.addData("Color3 toColor", robot.color3.getNormalizedColors().toColor());
            TELE.addData("Color3 green", green3 / (green3 + blue3 + red3));
            TELE.addData("Color3 distance (mm)", color3Distance);

            TELE.update();
        }
    }
}
