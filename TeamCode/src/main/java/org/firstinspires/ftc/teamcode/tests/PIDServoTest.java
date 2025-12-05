package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.PositionalServoProgrammer.restPos;
import static org.firstinspires.ftc.teamcode.utils.PositionalServoProgrammer.scalar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp
@Config
public class PIDServoTest extends LinearOpMode {

    public static double p = 0.0003, i = 0, d = 0.00001;

    public static double target = 0.5;

    Robot robot;

    double pos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        PIDController controller = new PIDController(p, i, d);

        controller.setTolerance(50);
        robot = new Robot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            controller.setPID(p, i, d);

            pos = scalar * ((robot.turr1Pos.getVoltage() - restPos) / 3.3);

            double pid = controller.calculate(pos, target);

            robot.turr1.setPosition(pid);
            robot.turr2.setPosition(-pid);

            telemetry.addData("pos", pos);
            telemetry.addData("target", target);
            telemetry.update();

        }

    }
}