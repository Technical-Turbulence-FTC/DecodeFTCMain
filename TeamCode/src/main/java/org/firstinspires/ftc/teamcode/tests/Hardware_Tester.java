package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Robot;

@Config
@TeleOp
public class Hardware_Tester extends LinearOpMode {
    Robot robot;
    MultipleTelemetry TELE;

    public static boolean subsystemMode = true;

    // Bare Motor Powers
    public static double flPow = 0;
    public static double frPow = 0;
    public static double blPow = 0;
    public static double brPow = 0;
    public static double intakePow = 0;
    public static double transferPow = 0;
    public static double shooter1Pow = 0;
    public static double shooter2Pow = 0;

    // Subsystem Motor Powers
    public static double drivetrainPow = 0;
    public static double shooterPow = 0;

    // Bare Servo Positions
    public static double spin1Pos = 0.501;
    public static double spin2Pos = 0.501;
    public static double turr1Pos = 0.501;
    public static double turr2Pos = 0.501;
    public static double transferServosPos = 0.501;
    public static double hoodPos = 0.501;
    public static double spindexBlockerPos = 0.501;
    public static double rapidFireBlockerPos = 0.501;
    public static double tilt1Pos = 0.501;
    public static double tilt2Pos = 0.501;

    // Subsystem Servo Positions
    public static double spinPos = 0.501;
    public static double turrPos = 0.501;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        TELE = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()){
            // Non-subsystem based components
            robot.setIntakePower(intakePow);
            robot.setTransferPower(transferPow);

            if (transferServosPos != 0.501){
                robot.setTransferServoPos(transferServosPos);
            }
            if (hoodPos != 0.501){
                robot.setHoodPos(hoodPos);
            }
            if (rapidFireBlockerPos != 0.501){
                robot.setRapidFireBlockerPos(rapidFireBlockerPos);
            }
            if (spindexBlockerPos != 0.501){
                robot.setSpindexBlockerPos(spindexBlockerPos);
            }
            if (tilt1Pos != 0.501){
                robot.setTilt1Pos(tilt1Pos);
            }
            if (tilt2Pos != 0.501){
                robot.setTilt2Pos(tilt2Pos);
            }

            // Subsystem based components
            if (subsystemMode){
                robot.setFrontLeftPower(drivetrainPow);
                robot.setFrontRightPower(drivetrainPow);
                robot.setBackLeftPower(drivetrainPow);
                robot.setBackRightPower(drivetrainPow);
                robot.shooter1.setPower(shooterPow);
                robot.shooter2.setPower(shooterPow);

                if (spinPos != 0.501){
                    robot.setSpinPos(spinPos);
                }
                if (turrPos != 0.501){
                    robot.setTurretPos(turrPos);
                }
            } else {
                robot.setFrontLeftPower(flPow);
                robot.setFrontRightPower(frPow);
                robot.setBackLeftPower(blPow);
                robot.setBackRightPower(brPow);
                robot.shooter1.setPower(shooter1Pow);
                robot.shooter2.setPower(shooter2Pow);

                if (spin1Pos != 0.501){
                    robot.spin1.setPosition(spin1Pos);
                }
                if (spin2Pos != 0.501){
                    robot.spin2.setPosition(spin2Pos);
                }
                if (turr1Pos != 0.501){
                    robot.turr1.setPosition(turr1Pos);
                }
                if (turr2Pos != 0.501){
                    robot.turr2.setPosition(turr2Pos);
                }
            }

            // Sensor Data

//            TELE.addData("Beam Break 1?", robot.beam1.isPressed());
//            TELE.addData("Beam Break 2?", robot.beam2.isPressed());
//            TELE.addData("Beam Break 3?", robot.beam3.isPressed());

            NormalizedRGBA revColor = robot.revSensor.getNormalizedColors();
            TELE.addData("REV Distance", robot.revSensor.getDistance(DistanceUnit.MM));
            TELE.addData("REV Green", revColor.green / (revColor.red + revColor.blue + revColor.green));

            TELE.addData("Voltage Sensor", robot.voltage.getVoltage());

            TELE.update();
        }
    }
}
