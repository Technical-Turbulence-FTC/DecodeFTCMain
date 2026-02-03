package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.constants.Color.redAlliance;
import static org.firstinspires.ftc.teamcode.constants.Poses.teleEnd;
import static org.firstinspires.ftc.teamcode.constants.Poses.teleStart;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.hoodOffset;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_in;
import static org.firstinspires.ftc.teamcode.constants.ServoPositions.transferServo_out;
import static org.firstinspires.ftc.teamcode.utils.Turret.turrDefault;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Robot;
import org.firstinspires.ftc.teamcode.utils.Servos;
import org.firstinspires.ftc.teamcode.utils.Spindexer;
import org.firstinspires.ftc.teamcode.utils.Targeting;
import org.firstinspires.ftc.teamcode.utils.Turret;

@Config
@Autonomous(preselectTeleOp = "TeleopV3")
public class Auto_LT_Far_3Ball extends LinearOpMode {
    public static double shoot0Vel = 3200, shoot0Hood = 0.5 + hoodOffset;
    public static double autoSpinStartPos = 0.2;
    public static double shoot0SpinSpeedIncrease = 0.015;

    public static double redObeliskTurrPos1 = turrDefault + 0.12;
    public static double redObeliskTurrPos2 = turrDefault + 0.13;
    public static double redObeliskTurrPos3 = turrDefault + 0.14;

    public static double blueObeliskTurrPos1 = turrDefault - 0.12;
    public static double blueObeliskTurrPos2 = turrDefault - 0.13;
    public static double blueObeliskTurrPos3 = turrDefault - 0.14;

    public static double shoot0XTolerance = 1.0;
    public static double redTurretShootPos = turrDefault;
    public static double blueTurretShootPos = turrDefault;
    public static int fwdTime = 200, strafeTime = 2300;
    public static double rPickupGateX = 1, rPickupGateY = 1, rPickupGateH = 1;
    public static double rPickupZoneX = 1, rPickupZoneY = 1, rPickupZoneH = 1;
    public static double rShootX = 1, rShootY = 1, rShootH = 1;
    public static double bPickupZoneX = 1, bPickupZoneY = 1, bPickupZoneH = 1;
    public static double bPickupGateX = 1, bPickupGateY = 1, bPickupGateH = 1;
    public static double bShootX = 1, bShootY = 1, bShootH = 1;
    public static int sleepTime = 1300;
    public int motif = 0;
    double turretShootPos = 0.0;
    Robot robot;
    MultipleTelemetry TELE;
    MecanumDrive drive;
    Servos servos;
    Spindexer spindexer;
    Flywheel flywheel;
    Turret turret;
    Targeting targeting;
    Targeting.Settings targetingSettings;
    private double firstSpindexShootPos = autoSpinStartPos;
    private boolean shootForward = true;
    private double xShoot, yShoot, hShoot;
    private int driverSlotGreen = 0;
    private int passengerSlotGreen = 0;
    private int rearSlotGreen = 0;
    private int mostGreenSlot = 0;
    private double pickupGateX = 0, pickupGateY = 0, pickupGateH = 0;
    private double pickupZoneX = 0, pickupZoneY = 0, pickupZoneH = 0;

    public Action shootAll(int vel, double shootTime, double spindexSpeed) {
        return new Action() {
            int ticker = 1;

            double stamp = 0.0;

            double velo = vel;

            int shooterTicker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

                flywheel.manageFlywheel(vel);
                velo = flywheel.getVelo();

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                robot.intake.setPower(-0.3);

                if (ticker == 1) {
                    stamp = getRuntime();
                }
                ticker++;

                robot.intake.setPower(0);
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                if (getRuntime() - stamp < shootTime) {

                    if (shooterTicker == 0 && !servos.spinEqual(autoSpinStartPos)) {
                        robot.spin1.setPosition(autoSpinStartPos);
                        robot.spin2.setPosition(1 - autoSpinStartPos);
                    } else {
                        robot.transferServo.setPosition(transferServo_in);
                        shooterTicker++;
                        double prevSpinPos = robot.spin1.getPosition();
                        robot.spin1.setPosition(prevSpinPos + spindexSpeed);
                        robot.spin2.setPosition(1 - prevSpinPos - spindexSpeed);
                    }

                    return true;

                } else {
                    robot.transferServo.setPosition(transferServo_out);
                    //spindexPos = spindexer_intakePos1;

                    spindexer.resetSpindexer();
                    spindexer.processIntake();

                    return false;

                }

            }
        };
    }

    public Action intake(double intakeTime) {
        return new Action() {
            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }
                ticker++;

                spindexer.processIntake();
                robot.intake.setPower(1);

                spindexer.ballCounterLight();
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

                return (System.currentTimeMillis() - stamp) < (intakeTime * 1000);

            }
        };
    }

    public Action manageFlywheel(
            double vel,
            double hoodPos,
            double time,
            double posX,
            double posY,
            double posXTolerance,
            double posYTolerance
    ) {

        boolean timeFallback = (time != 0.501);
        boolean posXFallback = (posX != 0.501);
        boolean posYFallback = (posY != 0.501);

        return new Action() {

            double stamp = 0.0;
            int ticker = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                drive.updatePoseEstimate();
                Pose2d currentPose = drive.localizer.getPose();

                if (ticker == 0) {
                    stamp = System.currentTimeMillis();
                }

                ticker++;

                flywheel.manageFlywheel(vel);
                robot.hood.setPosition(hoodPos);

                boolean timeDone = timeFallback && (System.currentTimeMillis() - stamp) > time * 1000;
                boolean xDone = posXFallback && Math.abs(currentPose.position.x - posX) < posXTolerance;
                boolean yDone = posYFallback && Math.abs(currentPose.position.y - posY) < posYTolerance;

                boolean shouldFinish = timeDone || xDone || yDone;
                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();
                TELE.addData("Velocity", flywheel.getVelo());
                TELE.addData("Hood", robot.hood.getPosition());
                TELE.update();

                return !shouldFinish;

            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        TELE = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry()
        );

        flywheel = new Flywheel(hardwareMap);

        targeting = new Targeting();
        targetingSettings = new Targeting.Settings(0.0, 0.0);

        spindexer = new Spindexer(hardwareMap);

        servos = new Servos(hardwareMap);

        robot.limelight.start();

        robot.limelight.pipelineSwitch(1);

        turret = new Turret(robot, TELE, robot.limelight);

        turret.manualSetTurret(turrDefault);

        drive = new MecanumDrive(hardwareMap, teleEnd);

        robot.spin1.setPosition(autoSpinStartPos);
        robot.spin2.setPosition(1 - autoSpinStartPos);

        robot.transferServo.setPosition(transferServo_out);

        robot.light.setPosition(1);

        while (opModeInInit()) {

            robot.hood.setPosition(shoot0Hood);
            turret.manualSetTurret(turretShootPos);

            if (gamepad2.crossWasPressed()) {
                redAlliance = !redAlliance;
            }

            if (gamepad2.dpadLeftWasPressed()) {
                turrDefault -= 0.01;
            }

            if (gamepad2.dpadRightWasPressed()) {
                turrDefault += 0.01;
            }

            redObeliskTurrPos1 = turrDefault + 0.12;
            redObeliskTurrPos2 = turrDefault + 0.13;
            redObeliskTurrPos3 = turrDefault + 0.14;

            blueObeliskTurrPos1 = turrDefault - 0.12;
            blueObeliskTurrPos2 = turrDefault - 0.13;
            blueObeliskTurrPos3 = turrDefault - 0.14;

            redTurretShootPos = turrDefault + 0.05;

            if (redAlliance) {
                robot.light.setPosition(0.28);

                pickupGateX = rPickupGateX;
                pickupGateY = rPickupGateY;
                pickupGateH = rPickupGateH;

                pickupZoneX = rPickupZoneX;
                pickupZoneY = rPickupZoneY;
                pickupZoneH = rPickupZoneH;

                xShoot = rShootX;
                yShoot = rShootY;
                hShoot = rShootH;

                turretShootPos = redTurretShootPos;

            } else {
                robot.light.setPosition(0.6);

                pickupGateX = bPickupGateX;
                pickupGateY = bPickupGateY;
                pickupGateH = bPickupGateH;

                pickupZoneX = bPickupZoneX;
                pickupZoneY = bPickupZoneY;
                pickupZoneH = bPickupZoneH;

                xShoot = bShootX;
                yShoot = bShootY;
                hShoot = bShootH;

                turretShootPos = blueTurretShootPos;

            }

            TELE.addData("Red?", redAlliance);
            TELE.addData("Turret Default", turrDefault);

            TELE.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {

            robot.transfer.setPower(1);

            Actions.runBlocking(
                    new ParallelAction(
                            manageFlywheel(
                                    shoot0Vel,
                                    shoot0Hood,
                                    9,
                                    0.501,
                                    0.501,
                                    shoot0XTolerance,
                                    0.501
                            )

                    )
            );

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            Actions.runBlocking(
                    shootAll((int) shoot0Vel, 6, shoot0SpinSpeedIncrease)
            );

            robot.frontLeft.setPower(-0.4);
            robot.frontRight.setPower(-0.4);
            robot.backLeft.setPower(-0.4);
            robot.backRight.setPower(-0.4);

            sleep(fwdTime);

            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            sleep(sleepTime);

            robot.frontLeft.setPower(-0.4);
            robot.frontRight.setPower(0.4);
            robot.backLeft.setPower(0.4);
            robot.backRight.setPower(-0.4);

            drive.updatePoseEstimate();

            teleStart = drive.localizer.getPose();

            sleep(strafeTime);

            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            while (opModeIsActive()) {

                drive.updatePoseEstimate();

                teleStart = drive.localizer.getPose();

                TELE.addLine("finished");
                TELE.update();
            }

        }

    }
}