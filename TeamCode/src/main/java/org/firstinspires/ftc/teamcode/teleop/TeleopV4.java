package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilsv2.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp
@Config
public class TeleopV4 extends LinearOpMode {
    Robot robot;
    Drivetrain drivetrain;
    MultipleTelemetry TELE;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(
                FtcDashboard.getInstance().getTelemetry(), telemetry
        );

        drivetrain = new Drivetrain(robot, TELE);

        drivetrain.setTelemetry(true);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){

            //Drivetrain

            drivetrain.drive(
                    -gamepad1.right_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_stick_x
            );

            TELE.update();
        }

    }
}
