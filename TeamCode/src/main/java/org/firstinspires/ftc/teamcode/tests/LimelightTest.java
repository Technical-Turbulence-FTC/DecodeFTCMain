package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.hardware.android.Rev3328;

import java.util.List;

@Config
@TeleOp
//TODO: fix to get the apriltag that it is reading
public class LimelightTest extends LinearOpMode {
    MultipleTelemetry TELE;
    public static int pipeline = 0; //0 is for test; 1 for obelisk; 2 is for blue track; 3 is for red track
    public static int mode = 0; //0 for bare testing, 1 for obelisk, 2 for blue track, 3 for red track
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.pipelineSwitch(pipeline);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot imuOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(imuOrientation));
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
                limelight.pipelineSwitch(1);
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId();
                        TELE.addData("ID", id);
                        TELE.update();
                    }

                }
            } else if (mode == 2 || mode == 3){
                if (mode == 2){
                    limelight.pipelineSwitch(2);
                } else {
                    limelight.pipelineSwitch(3);
                }

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(orientation.getYaw());
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        TELE.addData("tx", result.getTx());
                        TELE.addData("ty", result.getTy());
                        // MegaTag2 code for receiving position
                        Pose3D botpose = result.getBotpose_MT2();
                        if (botpose != null){
                            double x = botpose.getPosition().x;
                            double y = botpose.getPosition().y;
                            TELE.addData("Position X", x);
                            TELE.addData("Position Y", y);
                        }

                        TELE.update();
                    }
                }
            } else {
                limelight.pipelineSwitch(0);
            }
        }
    }
}
