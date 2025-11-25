package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Robot;

public class TrackingTest extends LinearOpMode {

    Robot robot;

    Drivetrain drivetrain;

    MultipleTelemetry TELE;

    GamepadEx g1;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        g1 = new GamepadEx(gamepad1);

        drivetrain = new Drivetrain(robot, TELE, g1);


        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive()){
            drivetrain.update();
        }

    }
}
