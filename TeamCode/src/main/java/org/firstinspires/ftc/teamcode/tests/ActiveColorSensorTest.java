package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;


@TeleOp
@Config
public class ActiveColorSensorTest extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int b1Purple = 1;
        int b1Total = 1;
        int b2Purple = 1;
        int b2Total = 1;
        int b3Purple = 1;
        int b3Total = 1;

        double totalStamp1 = 0.0;
        double purpleStamp1 = 0.0;
        double totalStamp2 = 0.0;
        double purpleStamp2 = 0.0;
        double totalStamp3 = 0.0;
        double purpleStamp3 = 0.0;

        String b1 = "none";
        String b2 = "none";
        String b3 = "none";

        double position = 0.0;

        double stamp = getRuntime();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            if ((getRuntime() % 0.3) >0.15) {
                position = spindexer_intakePos1 + 0.015;
            } else {
                position = spindexer_intakePos1 - 0.015;
            }
            robot.spin1.setPosition(position);
            robot.spin2.setPosition(1-position);

            robot.intake.setPower(1);

            if (robot.pin1.getState()){
                if (robot.pin0.getState()){
                    b1Purple += 1;
                    purpleStamp1 = getRuntime();
                }
                b1Total += 1;
                totalStamp1 = getRuntime();
            }
            if (getRuntime() - totalStamp1 > 1){
                b1 = "none";
                b1Total = 1;
                b1Purple = 1;
            }else if (b1Total > 10 && getRuntime() - purpleStamp1 > 2){
                b1 = "Green";
            }else if (b1Purple > 10){
                b1 = "Purple";
            }

            if (robot.pin3.getState()){
                if (robot.pin2.getState()){
                    b2Purple += 1;
                    purpleStamp2 = getRuntime();
                }
                b2Total += 1;
                totalStamp2 = getRuntime();
            }
            if (getRuntime() - totalStamp2 > 1){
                b2 = "none";
                b2Total = 1;
                b2Purple = 1;
            }else if (b2Total > 10 && getRuntime() - purpleStamp2 > 2){
                b2 = "Green";
            }else if (b2Purple > 10){
                b2 = "Purple";
            }

            if (robot.pin5.getState()){
                if (robot.pin4.getState()){
                    b3Purple += 1;
                    purpleStamp3 = getRuntime();
                }
                b3Total += 1;
                totalStamp3 = getRuntime();
            }
            if (getRuntime() - totalStamp3 > 1){
                b3 = "none";
                b3Total = 1;
                b3Purple = 1;
            }else if (b3Total > 10 && getRuntime() - purpleStamp3 > 2){
                b3 = "Green";
            }else if (b3Purple > 10){
                b3 = "Purple";
            }

            TELE.addData("Green1:", robot.pin1.getState());
            TELE.addData("Purple1:", robot.pin0.getState());
            TELE.addData("Green2:", robot.pin3.getState());
            TELE.addData("Purple2:", robot.pin2.getState());
            TELE.addData("Green3:", robot.pin5.getState());
            TELE.addData("Purple3:", robot.pin4.getState());
            TELE.addData("1", b1);
            TELE.addData("2",b2);
            TELE.addData("3",b3);

            TELE.update();
        }
    }

}