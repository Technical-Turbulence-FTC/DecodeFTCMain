package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Robot;

import java.util.ArrayList;
import java.util.List;

public class TeleopV2 extends LinearOpMode {

    Robot robot;
    MultipleTelemetry TELE;

    boolean intake = false;
    boolean reject = false;

    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    List<Double> s1 = new ArrayList<>();
    List<Double> s2 = new ArrayList<>();
    List<Double> s3 = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            //DRIVETRAIN:

            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backRight.setPower(backRightPower);

            //INTAKE:

            if (gamepad1.rightBumperWasPressed()) {
                intake = true;
            }

            if (intake) {
                robot.intake.setPower(1);
            } else if (reject) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }


            //COLOR:

            double s1D = robot.

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            TELE.update();

        }
    }
}
