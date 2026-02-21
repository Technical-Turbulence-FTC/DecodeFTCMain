package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Front_Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.hoodOffset;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spinStartPos;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_intakePos1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.spindexer_outtakeBall1;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.teleop.TeleopV3.spinSpeedIncrease;
import static org.firstinspires.ftc.teamcode.utils.Targeting.turretInterpolate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    public static int mode = 1;
    public static double parameter = 0.0;
    // --- CONSTANTS YOU TUNE ---

    public static double Velocity = 0.0;
    public static double P = 255.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 75;
    public static double transferPower = 1.0;
    public static double hoodPos = 0.501;
    public static double turretPos = 0.501;
    public static boolean shoot = false;

    public static boolean intake = false;
    public static boolean turretTrack = true;
    Robot robot;
    Flywheel flywheel;
    Servos servo;
    MecanumDrive drive;
    Turret turret;
    double shootStamp = 0.0;
    boolean shootAll = false;

    public double spinPow = 0.09;

    public static boolean enableHoodAutoOpen = false;
    public double hoodAdjust = 0.0;
    public static double hoodAdjustFactor = 1.0;
    private int shooterTicker = 0;
    public static double spinSpeed = 0.02;
    Spindexer spindexer ;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        DcMotorEx leftShooter = robot.shooter1;
        DcMotorEx rightShooter = robot.shooter2;
        flywheel = new Flywheel(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        servo = new Servos(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        MultipleTelemetry TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );
        turret = new Turret(robot, TELE, robot.limelight);
        Turret.limelightUsed = true;

        waitForStart();

        robot.limelight.start();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (redAlliance){
                robot.limelight.pipelineSwitch(4);
            } else {
                robot.limelight.pipelineSwitch(2);
            }

            //TURRET TRACKING
            drive.updatePoseEstimate();

            double robX = drive.localizer.getPose().position.x;
            double robY = drive.localizer.getPose().position.y;

            double robotHeading = drive.localizer.getPose().heading.toDouble();

            double goalX = -15;
            double goalY = 0;

            double dx = robX - goalX;  // delta x from robot to goal
            double dy = robY - goalY;  // delta y from robot to goal
            Pose2d deltaPose = new Pose2d(dx, dy, robotHeading);

            double distanceToGoal = Math.sqrt(dx * dx + dy * dy);

            if (turretTrack){
                turret.trackGoal(deltaPose);
            } else if (turretPos != 0.501){
                turret.setTurret(turretPos);
            }

            double voltage = robot.voltage.getVoltage();

            if (mode == 0) {
                rightShooter.setPower(parameter);
                leftShooter.setPower(parameter);
            } else if (mode == 1) {
                flywheel.setPIDF(P, I, D, F / voltage);
                flywheel.manageFlywheel((int) Velocity);
            }

            if (hoodPos != 0.501) {
                if (enableHoodAutoOpen) {
                    robot.hood.setPosition(hoodPos+(hoodAdjustFactor*(flywheel.getVelo()/Velocity)) + hoodOffset);
                } else {
                    robot.hood.setPosition(hoodPos + hoodOffset);
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

                //spindexPos = spindexer_intakePos1;
                if (getRuntime() - shootStamp < 3.5) {

                    if (shooterTicker == 0 && !servo.spinEqual(spinStartPos)){
                        robot.spin1.setPosition(spinStartPos);
                        robot.spin2.setPosition(1-spinStartPos);
                    } else {
                        robot.transferServo.setPosition(transferServo_in);
                        shooterTicker++;
                        double prevSpinPos = robot.spin1.getPosition();

                        if (prevSpinPos < 0.9){
                            robot.spin1.setPosition(prevSpinPos + spinSpeed);
                            robot.spin2.setPosition(1 - prevSpinPos - spinSpeed);
                        }
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
            TELE.addData("Voltage", voltage);

            TELE.update();

        }

    }
}
