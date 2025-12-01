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

            // Reset the counters after 1 second of not reading a ball.
            final double ColorCounterResetDelay = 1.0;
            // Number of times the loop needs to run before deciding on a color.
            final int ColorCounterTotalMinCount = 20;
            // If the color sensor reads a color this percentage of time
            // out of the total, declare the color.
            // Usage: (Color Count)/(Total Count) > ColorCounterThreshold
            final double ColorCounterThreshold  = 0.65;

            if (robot.pin1.getState()){
                if (robot.pin0.getState()){
                    b1Purple ++;
                }
                b1Total++;
                totalStamp1 = getRuntime();
            }
            if (getRuntime() - totalStamp1 > ColorCounterResetDelay) {
                // Too Much time has passed without detecting ball
                b1 = "none";
                b1Total = 1;
                b1Purple = 1;
            }else if ((b1Total > ColorCounterTotalMinCount) && ((double) b1Purple / b1Total) >= ColorCounterThreshold){
            // Enough Time has passed and we met the threshold
                b1 = "Purple";
            }else if (b1Total > ColorCounterTotalMinCount) {
            // Enough Time passed WITHOUT meeting the threshold
                b1 = "Green";
            }

            if (robot.pin3.getState()){
                if (robot.pin2.getState()){
                    b2Purple ++;
                }
                b2Total++;
                totalStamp2 = getRuntime();
            }
            if (getRuntime() - totalStamp2 > ColorCounterResetDelay) {
                // Too Much time has passed without detecting ball
                b2 = "none";
                b2Total = 1;
                b2Purple = 1;
            }else if ((b2Total > ColorCounterTotalMinCount) && ((double) b2Purple / b2Total) >= ColorCounterThreshold){
                // Enough Time has passed and we met the threshold
                b2 = "Purple";
            }else if (b2Total > ColorCounterTotalMinCount) {
                // Enough Time passed WITHOUT meeting the threshold
                b2 = "Green";
            }

            if (robot.pin5.getState()){
                if (robot.pin4.getState()){
                    b3Purple ++;
                }
                b3Total++;
                totalStamp3 = getRuntime();
            }
            if (getRuntime() - totalStamp3 > ColorCounterResetDelay) {
                // Too Much time has passed without detecting ball
                b3 = "none";
                b3Total = 1;
                b3Purple = 1;
            }else if ((b3Total > ColorCounterTotalMinCount) && ((double) b3Purple / b3Total) >= ColorCounterThreshold){
                // Enough Time has passed and we met the threshold
                b3 = "Purple";
            }else if (b3Total > ColorCounterTotalMinCount) {
                // Enough Time passed WITHOUT meeting the threshold
                b3 = "Green";
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