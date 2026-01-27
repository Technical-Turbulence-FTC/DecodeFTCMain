package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.teleop.TeleopV3.spinSpeedIncrease;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    public static int mode = 1;
    public static double parameter = 0.0;
    // --- CONSTANTS YOU TUNE ---

    //TODO: @Daniel FIX THE BELOW CONSTANTS A LITTLE IF NEEDED
    public static double Velocity = 0.0;
    public static double P = 255.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 7.5;
    public static double transferPower = 1.0;
    public static double hoodPos = 0.501;
    public static double turretPos = 0.501;
    public static boolean shoot = false;

    public static boolean intake = false;
    Robot robot;
    Flywheel flywheel;
    Servos servo;

    double shootStamp = 0.0;
    boolean shootAll = false;

    public double spinPow = 0.09;

    public static boolean enableHoodAutoOpen = false;
    public double hoodAdjust = 0.0;
    public static double hoodAdjustFactor = 1.0;
    private int shooterTicker = 0;
    Spindexer spindexer ;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        DcMotorEx leftShooter = robot.shooter1;
        DcMotorEx rightShooter = robot.shooter2;
        flywheel = new Flywheel(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        servo = new Servos(hardwareMap);

        MultipleTelemetry TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (mode == 0) {
                rightShooter.setPower(parameter);
                leftShooter.setPower(parameter);
            } else if (mode == 1) {
                flywheel.setPIDF(P, I, D, F);
                flywheel.manageFlywheel((int) Velocity);
            }

            if (hoodPos != 0.501) {
                if (enableHoodAutoOpen) {
                    robot.hood.setPosition(hoodPos+(hoodAdjustFactor*(flywheel.getVelo()/Velocity)));
                } else {
                    robot.hood.setPosition(hoodPos);
                }
            }

            if (intake) {
                robot.intake.setPower(1);

            } else {
                robot.intake.setPower(0);
            }


            if (shoot) {
                shootStamp = getRuntime();
                shootAll = true;
                shoot = false;
                robot.transfer.setPower(transferPower);
                shooterTicker = 0;
            }
            if (shootAll) {

                //intake = false;
                //reject = false;

                // TODO: Change starting position based on desired order to shoot green ball
                //spindexPos = spindexer_intakePos1;
                if (getRuntime() - shootStamp < 3.5) {

                    if (shooterTicker == 0 && !servo.spinEqual(spinStartPos)){
                        robot.spin1.setPosition(spinStartPos);
                        robot.spin2.setPosition(1-spinStartPos);
                    } else {
                        robot.transferServo.setPosition(transferServo_in);
                        shooterTicker++;
                        double prevSpinPos = robot.spin1.getPosition();
                        robot.spin1.setPosition(prevSpinPos + spinSpeedIncrease);
                        robot.spin2.setPosition(1 - prevSpinPos - spinSpeedIncrease);
                    }



                } else {
                    robot.transferServo.setPosition(transferServo_out);
                    //spindexPos = spindexer_intakePos1;

                    shootAll = false;
                    shooterTicker = 0;

                    robot.transferServo.setPosition(transferServo_out);
                    robot.transfer.setPower(0);


                    spindexer.resetSpindexer();
                    spindexer.processIntake();
                }
            } else {
                spindexer.processIntake();
            }

            TELE.addData("Velocity", flywheel.getVelo());
            TELE.addData("Velocity 1", flywheel.getVelo1());
            TELE.addData("Velocity 2", flywheel.getVelo2());
            TELE.addData("Power", robot.shooter1.getPower());
            TELE.addData("Steady?", flywheel.getSteady());
            TELE.addData("Position", robot.shooter1.getCurrentPosition());

            TELE.update();

        }

    }
}
