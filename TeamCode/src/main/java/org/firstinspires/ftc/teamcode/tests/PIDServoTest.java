package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.Robot;

@TeleOp
@Config
public class PIDServoTest extends LinearOpMode {

    public static double p = 2, i = 0, d = 0, f = 0;

    public static double target = 0.5;

    public static int mode = 0; //0 is for turret, 1 is for spindexer

    public static double scalar = 1.01;
    public static double restPos = 0.0;

    Robot robot;

    double pos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        PIDFController controller = new PIDFController(p, i, d, f);

        controller.setTolerance(0);
        robot = new Robot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            controller.setPIDF(p, i, d, f);

            if (mode == 0) {
                pos = scalar * ((robot.turr1Pos.getVoltage() - restPos) / 3.3);

                double pid = controller.calculate(pos, target);

                robot.turr1.setPower(pid);
                robot.turr2.setPower(-pid);
            } else if (mode == 1) {
                pos = scalar * ((robot.spin1Pos.getVoltage() - restPos) / 3.3);

                double pid = controller.calculate(pos, target);

                robot.spin1.setPower(pid);
                robot.spin2.setPower(-pid);
            }

            telemetry.addData("pos", pos);
            telemetry.addData("Turret Voltage", robot.turr1Pos.getVoltage());
            telemetry.addData("Spindex Voltage", robot.spin1Pos.getVoltage());
            telemetry.addData("target", target);
            telemetry.addData("Mode", mode);
            telemetry.update();

        }

    }
}