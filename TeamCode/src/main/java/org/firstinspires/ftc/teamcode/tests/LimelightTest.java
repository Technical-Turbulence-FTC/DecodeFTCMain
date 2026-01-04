package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.List;

public class LimelightTest extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;
    public static int pipeline = 0; //0 is for test; 1, 2, and 3 are for obelisk; 4 is for blue track; 5 is for red track
    public static int mode = 0; //0 for bare testing, 1 for obelisk, 2 for blue track, 3 for red track
    int obeliskPipe = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.limelight.pipelineSwitch(pipeline);

        waitForStart();
        if (isStopRequested()) return;
        robot.limelight.start();
        while (opModeIsActive()){
            if (mode == 0){
                robot.limelight.pipelineSwitch(pipeline);
                LLResult result = robot.limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        TELE.addData("tx", result.getTx());
                        TELE.addData("ty", result.getTy());
                        TELE.update();
                    }
                }
            } else if (mode == 1){
                robot.limelight.pipelineSwitch(obeliskPipe);
                LLResult result = robot.limelight.getLatestResult();
                if (result != null && result.isValid()){
                    TELE.addData("ID", obeliskPipe+20);
                    TELE.update();
                } else {
                    if (obeliskPipe >= 3){
                        obeliskPipe = 1;
                    } else {
                        obeliskPipe++;
                    }
                }
            } else if (mode == 2){
                robot.limelight.pipelineSwitch(4);
                LLResult result = robot.limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        TELE.addData("tx", result.getTx());
                        TELE.addData("ty", result.getTy());
                        TELE.update();
                    }
                }
            } else if (mode == 3){
                robot.limelight.pipelineSwitch(5);
                LLResult result = robot.limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        TELE.addData("tx", result.getTx());
                        TELE.addData("ty", result.getTy());
                        TELE.update();
                    }
                }
            } else {
                robot.limelight.pipelineSwitch(0);
            }
        }
    }
}
