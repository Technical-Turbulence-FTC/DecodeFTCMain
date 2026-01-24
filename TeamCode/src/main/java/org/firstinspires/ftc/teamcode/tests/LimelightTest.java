package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Config
@TeleOp
//TODO: fix to get the apriltag that it is reading
public class LimelightTest extends LinearOpMode {
    MultipleTelemetry TELE;
    Turret turret;
    Robot robot;
    public static int pipeline = 0; //0 is for test; 1 for obelisk; 2 is for blue track; 3 is for red track
    public static int mode = 0; //0 for bare testing, 1 for obelisk, 2 for blue track, 3 for red track
    @Override
    public void runOpMode() throws InterruptedException {
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap);
        turret = new Turret(robot, TELE, robot.limelight);
        robot.limelight.pipelineSwitch(pipeline);
        waitForStart();
        if (isStopRequested()) return;
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
                int obeliskID = turret.detectObelisk();
                TELE.addData("Limelight ID", obeliskID);
                TELE.update();
            } else if (mode == 2 || mode == 3){ // Use redAlliance variable to switch between red and blue
                double tx = turret.getBearing();
                double ty = turret.getTy();
                double x = turret.getLimelightX();
                double y = turret.getLimelightY();
                TELE.addData("tx", tx);
                TELE.addData("ty", ty);
                TELE.addData("x", x);
                TELE.addData("y", y);
                TELE.update();
            } else {
                robot.limelight.pipelineSwitch(0);
            }
        }
    }
}
