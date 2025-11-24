package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@TeleOp
public class TransferTest extends LinearOpMode {

    Robot robot;


    Transfer transfer;

    public static double pos = 0.40;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        transfer = new Transfer(robot);

        waitForStart();

        while (opModeIsActive()){
            transfer.setTransferPosition(pos);
        }

    }
}
