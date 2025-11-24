package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@TeleOp
public class MotorDirectionDebugger extends LinearOpMode {

    public static double flPower = 0.0;

    public static double frPower = 0.0;

    public static double blPower = 0.0;
    public static double brPower = 0.0;

    Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);


        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()){

            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);
            robot.backRight.setPower(brPower);
            robot.backLeft.setPower(blPower);



        }
    }
}
