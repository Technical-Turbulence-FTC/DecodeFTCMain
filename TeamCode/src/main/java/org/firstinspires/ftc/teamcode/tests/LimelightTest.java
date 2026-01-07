package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Robot;
//TODO: fix to get the apriltag that it is reading
public class LimelightTest extends LinearOpMode {
    MultipleTelemetry TELE;
    public static int pipeline = 0; //0 is for test; 1, 2, and 3 are for obelisk; 4 is for blue track; 5 is for red track
    public static int mode = 0; //0 for bare testing, 1 for obelisk, 2 for blue track, 3 for red track
    int obeliskPipe = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.pipelineSwitch(pipeline);
        waitForStart();
        if (isStopRequested()) return;
        limelight.start();
        while (opModeIsActive()){
            if (mode == 0){
                limelight.pipelineSwitch(pipeline);
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        TELE.addData("tx", result.getTx());
                        TELE.addData("ty", result.getTy());
                        TELE.update();
                    }
                }
            } else if (mode == 1){
                limelight.pipelineSwitch(obeliskPipe);
                LLResult result = limelight.getLatestResult();
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
                limelight.pipelineSwitch(4);
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        TELE.addData("tx", result.getTx());
                        TELE.addData("ty", result.getTy());
                        TELE.update();
                    }
                }
            } else if (mode == 3){
                limelight.pipelineSwitch(5);
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        TELE.addData("tx", result.getTx());
                        TELE.addData("ty", result.getTy());
                        TELE.update();
                    }
                }
            } else {
                limelight.pipelineSwitch(0);
            }
        }
    }
}
